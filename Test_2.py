from vcScript import *
import vcMatrix
import vcVector
import math
import vcHelpers.Robot2 as robohelp
from vcHelpers.Output import *
from collections import deque

app = getApplication()
comp = getComponent()
sim = getSimulation()
comp = getComponent()

APP_VER = int(app.ProductVersion.replace('.', ''))
IDENTITY_MTX = vcMatrix.new()
EPSILON = 0.1
DEBUG_MODE = 0
THIS_SCRIPT = comp.findBehaviour('ResourceScript')
wait_signal_triggerers = []
trigger_activated = False
trigger_signal = comp.findBehaviour('Trigger')
travel_controller = None
stats_manager = None
human_control = None
power_manager = None
obj_interp_manager = None
setDebug(True)

enable_pick_place_tolerance_prop = comp.getProperty("Transport::EnablePickPlaceTolerance")
move_tolerance_prop = comp.getProperty("Transport::MoveTolerance")


def OnAction(action):
    global mission_mode_on, trigger_activated
    trigger_activated = True
    if action.Name == 'Task':
        debug('Task received with message: {}'.format(action.Message), simtime=True)
        action_manager.read_tasks(action)
        trigger_signal.signal(True)
    elif action.Name == 'ExecuteMission':
        if not mission_mode_on:
            mission_mode_on = True
            debug_p.IsVisible = True
            debug('MISSION_MODE ON', simtime=True)
        debug('Mission received with message: {}'.format(action.Message), simtime=True)
        action_manager.read_mission(action)


def OnSignal(signal):
    global trigger_activated, triggerer, wait_signal_triggerers
    trigger_activated = True
    triggerer = signal
    evaluateCondition()

    # Added (Emergency Stop)
    if signal.Name == "EmergencyStop" and emergency_stop.WritableWhenSimulating:
        comp.EmergencyStop = signal.Value


def OnStart():
    global config
    global travel_controller
    global traffic_service
    global travel_scheduler
    global pattern_manager
    global actuator_control
    global stats_manager
    global power_manager
    global human_control
    global idle_locations, charging_stations
    global robo, robot_controller, robot_tool_iface, robot_home_joint_vals
    global current_tool
    global world_container
    global obj_interp_manager
    global collision_mgr
    global mission_mode_on
    global wait_signal_triggerers
    global triggerer
    global listened_occupied_properties

    listened_occupied_properties = []
    wait_signal_triggerers = []
    triggerer = None

    try:
        config = Config()
    except Exception as e:
        suspendRun()
        error(e)
        return

    actuator_control = None
    if config.actuator_control_enabled:
        actuator_control = ActuatorControl(config.cont.Parent)

    states = [
        ('Processing', VC_STATISTICS_BUSY, 'lime'),
        ('Repairing', VC_STATISTICS_BUSY, 'green'),
        ('Transporting', VC_STATISTICS_BUSY, 'green'),
        ('Picking', VC_STATISTICS_BUSY, 'pale_green'),
        ('Placing', VC_STATISTICS_BUSY, 'pale_green'),
        ('Moving', VC_STATISTICS_BUSY, 'pale_green'),
        ('CollectingTool', VC_STATISTICS_BUSY, 'light_cyan'),
        ('ReturningTool', VC_STATISTICS_BUSY, 'blue'),
        ('Idle', VC_STATISTICS_IDLE, 'yellow'),
        ('Blocked', VC_STATISTICS_BLOCKED, 'orange'),
        ('Bypassing', VC_STATISTICS_BLOCKED, 'orange'),
        ('Charging', VC_STATISTICS_SETUP, 'cyan'),
        ('Break', VC_STATISTICS_BREAK, 'black')
    ]
    stats_manager = StatisticsManager(comp.findNode('Status'))
    stats_manager.define_states(states)

    power_manager = PowerManager(stats_manager.statistics, config.power_manager_enabled)

    if power_manager.enabled:
        stats_manager.power_manager = power_manager

    pattern_manager = PatternManager()
    initialize_patterns(pattern_manager)

    world_container = sim.World.getBehavioursByType(VC_CONTAINER)[0]  # type: vcComponentContainer
    current_tool = get_first_cart()

    obj_interp_manager = ObjectInterpolatorManager()

    action_manager.initialize(lifo=comp.getProperty('Transport::LIFO').Value)
    if not action_manager.connected_controller:
        travel_controller = None
        suspendRun()
        return

    # Detach the resource if attached to anything
    if comp.Parent != comp.World and comp.Parent.Type != VC_LAYOUT:
        comp.World.attach(comp, True)
        comp.PositionMatrix = comp.WorldPositionMatrix
        comp.saveState()
        info('Component detached!')

    tc = action_manager.connected_controller
    update_signal = comp.findBehaviour('Update')  # type: vcStringSignal

    travel_controller = TravelController(comp.WorldPositionMatrix, tc, update_signal)
    travel_controller.default_turn_speed = comp.TurnSpeed

    traffic_service = TrafficService(tc, update_signal)
    collision_mgr = CollisionManager(action_manager.connected_controller)
    travel_scheduler = TravelScheduler(travel_controller, traffic_service, collision_mgr)
    travel_controller.visible_to_traffic = collision_mgr.state != CollisionManager.STATE_DISABLED

    human_control = None
    if config.human_control_enabled:
        human_control = HumanAnimationControl(travel_controller)

    connected_locations = tc.findBehaviour('Idle/ChargingLocations').ConnectedComponents
    idle_locations = [c for c in connected_locations if
                      not c.getProperty('AllowIdling') or c.getProperty('AllowIdling') and c.AllowIdling]
    charging_stations = []
    if config.power_manager_enabled:
        charging_stations = [c for c in connected_locations if c.getProperty('Charger') and c.Charger]

    # check do we have robot arm onboard (attached)
    robo = None
    robot_controller = None
    robot_tool_iface = None
    if not action_manager.mounted_tcs:
        for child in comp.ChildComponents:
            if child.findBehavioursByType(VC_ROBOTCONTROLLER):
                robo = robohelp.getRobot(child)
                robot_controller = robo.Controller
                robot_tool_iface = robot_controller.FlangeNode.getBehavioursByType(VC_ONETOONEINTERFACE)[0]
                update_arm_config_prop_list(robo)
                robot_home_joint_vals = [j.CurrentValue for j in robo.Joints]
                break
    toggle_robot_arm_tab(robo)

    on_availability_changed(available_p)  # set initial availability
    if available_p.Value:
        stats_manager.set_state('Idle')

    OnSimulationLevelChanged(comp.SimulationLevel)

    mission_mode_on = False

    if transport_product_orientation:
        transport_product_orientation.Value = 0.0

    # Added (Emergency Stop)
    comp.EmergencyStop = False
    emergency_stop_gui(False)
    global manual_pause
    manual_pause = False

    # Added (FABRIK)
    if looks:
        comp.Looks = "Otto"


def OnRun():
    global timeout_enabled, last_idle_chrg_loc
    travel_controller.load_schemas()
    travel_controller.load_areas()
    collision_mgr.initialize()

    timeout_enabled = True
    last_idle_chrg_loc = None

    time_to_idle = comp.TimeToIdle
    while True:

        if power_manager.enabled and stats_manager.statistics.State == 'Charging':
            idle_timeout = power_manager.get_time_to_fullcharge()
        else:
            idle_timeout = 0.0 if not timeout_enabled else time_to_idle
        actions_available = condition(action_manager.actions_available, idle_timeout)
        timeout_enabled = True

        if actions_available or last_idle_chrg_loc not in idle_locations:  # in case charging at idle location
            release_idle_or_charging_location()

        if actions_available:
            action_manager.execute_next_action()
        elif action_manager.is_reserved():  # wait for release or new actions
            debug('is_reserved.')

            action_manager.reserved_to_p.OnChanged = activate_trigger
            triggerCondition(lambda: action_manager.actions_available() or not action_manager.is_reserved())
            action_manager.reserved_to_p.OnChanged = None

            if action_manager.actions_available():
                action_manager.execute_next_action()
            else:
                action_manager.execute_idle_action()
        else:
            action_manager.execute_idle_action()


def OnSimulationUpdate(sim_time):
    if sim.IsRunning and travel_controller:
        if power_manager:
            power_manager.update()
        comp.PositionMatrix = travel_controller.interpolate(sim_time)
        obj_interp_manager.interpolate(sim_time)
        if human_control:
            human_control.animate()
        comp.update()


def OnSimulationLevelChanged(level):
    if human_control:
        human_control.on_sim_lvl_change(level)


def OnReset():
    if stats_manager:
        stats_manager.set_state('Idle')
    if travel_controller:
        travel_controller.unsubscribe_listeners()

    transport_product_lift = comp.getProperty("Transport::ProductLift")
    if transport_product_lift:
        transport_product_lift.Value = 0.0  # Added (KIVA)

    if transport_product_orientation:
        transport_product_orientation.Value = 0.0


def OnFinalize():
    global config
    config = Config()
    if config.human_control_enabled:
        HumanAnimationControl.hide_joint_properties()


############# IMPLEMENTATIONS ################
def execute_pick_action(manager, task_action):
    # type: (ActionManager, vcAction) -> None
    global current_tool

    speed = comp.MoveSpeedLoaded if config.cont.ComponentCount else comp.MoveSpeed
    # debug('*** Executing ({}) action with properties {}'.format(task_action.Name, [(p.Name, p.Value) for p in task_action.Properties]), simtime=True)

    component = task_action.Component

    # Check tool
    tool_comp = task_action.ToolComponent
    if not tool_comp and current_tool and task_action.TransportLink.Tool == 'Use Current':
        tool_comp = current_tool

    # Check if tool is a cart, confirm which cart is correct one in the train
    assigned_cart = None
    if tool_comp and is_cart(tool_comp):
        assigned_cart = assing_to_cart(tool_comp, component, task_action.TransportLink.Tool,
                                       task_action.TransportLink.ToolName)
        if assigned_cart:
            tool_comp = assigned_cart
            if assigned_cart in comp.ChildComponents:
                current_tool = assigned_cart

    # Check if current tool needs to be returned
    if current_tool and (task_action.TransportLink.Tool == 'No Tool' or \
                         task_action.TransportLink.Tool != 'Use Current' and tool_comp != current_tool or \
                         # not assigned_cart and task_action.TransportLink.Tool != 'Use Current' and tool_comp != current_tool or \
                         # assigned_cart and task_action.TransportLink.Tool != 'Use Current' and not assigned_cart in comp.ChildComponents or \
                         manager.assistance in (ActionManager.LOAD_ASSISTANT, ActionManager.UNLOAD_ASSISTANT)):
        return_tool()

    # Check if assigned tool needs to be collected
    if not current_tool and tool_comp and manager.assistance not in (
    ActionManager.LOAD_ASSISTANT, ActionManager.UNLOAD_ASSISTANT):
        collect_tool(tool_comp, speed)

    source_node = task_action.SourceNode
    source_node, transport_target = get_transport_data(component, source_node, source=True)
    offset_vec_p = task_action.TransportLink.getProperty('ResourcePickOffset')
    offset_vec = offset_vec_p.Value if offset_vec_p else vcVector.new()
    approach_vec_p = task_action.TransportLink.getProperty('PickApproach')
    approach_vec = approach_vec_p.Value if approach_vec_p else vcVector.new()
    approach_distance = approach_vec.length()
    move_target = transport_target.ResourcePosition  # type: vcMatrix
    move_target.translateRel(offset_vec.X, offset_vec.Y, offset_vec.Z)
    cart_offset = get_cart_offset(assigned_cart)
    if cart_offset:
        move_target.translateRel(cart_offset.X, cart_offset.Y, cart_offset.Z)
    travel_target = vcMatrix.new(move_target)
    travel_target.translateRel(approach_vec.X, approach_vec.Y, approach_vec.Z)

    assisted = False
    assistant = False
    if manager.assistance == ActionManager.LOAD_ASSISTANT:
        # pick from process
        # print comp.Name, 'assist: pick from process'
        assistant = True
        assist_frame = source_node.Component.getFeature('AssistLocation')
        if assist_frame:
            move_target = source_node.Component.WorldPositionMatrix * assist_frame.NodePositionMatrix
            travel_target = move_target
        else:
            use_resourceloc_p = task_action.AssistComponent.getProperty('Assist::UseResourceLocation')
            if not use_resourceloc_p or not use_resourceloc_p.Value:
                component.update()
                move_target = assist_resolve_move_target(transport_target.ResourcePosition,
                                                         component.WorldPositionMatrix)
            travel_target = move_target
    elif manager.assistance == ActionManager.UNLOAD_ASSISTANT:
        # pick from assisted resource
        # print comp.Name, 'assist: pick from assisted resource'
        assistant = True
        destination_node, transport_target = get_transport_data(component, source_node, source=False)
        move_target = transport_target.ResourcePosition  # type: vcMatrix
        offset_vec_p = task_action.TransportLink.getProperty('ResourcePlaceOffset')
        offset_vec = offset_vec_p.Value if offset_vec_p else vcVector.new()
        move_target.translateRel(offset_vec.X, offset_vec.Y, offset_vec.Z)
        use_resourceloc_p = task_action.AssistComponent.getProperty('Assist::UseResourceLocation')
        if not use_resourceloc_p or not use_resourceloc_p.Value:
            move_target = assist_resolve_move_target(transport_target.ProductPosition, move_target)
        travel_target = move_target
    elif manager.assistance == ActionManager.LOAD_ASSISTED:
        assisted = True
        assist_requested = assist_request_assistance(task_action, load=True, to_mounted=False)

    if assistant:
        fasten_only_p = task_action.AssistComponent.getProperty('Assist::FastenOnly')
        assist_fasten_only = fasten_only_p.Value if fasten_only_p else False
    else:
        assist_fasten_only = False

    comp_wpm = travel_controller.interpolate(sim.SimTime)
    travel_threshold = approach_distance if approach_distance > 0.0 else 200.0
    distance_to_target = (comp_wpm.P - move_target.P).length()
    angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)

    enable_pick_place_tolerance_prop = comp.getProperty("Transport::EnablePickPlaceTolerance")
    move_tolerance_prop = comp.getProperty("Transport::MoveTolerance")

    if enable_pick_place_tolerance_prop and move_tolerance_prop:
        enable_pick_place_tolerance = enable_pick_place_tolerance_prop.Value
        move_tolerance = move_tolerance_prop.Value
    else:
        enable_pick_place_tolerance = False

    if enable_pick_place_tolerance and distance_to_target < move_tolerance:
        travel_controller.allow_reverse = False
        hdg = travel_controller.solve_move_heading(comp_wpm, travel_target)
        rotate_to_release = travel_controller.single_move(comp_wpm.P, comp.MoveSpeedApproach, comp.TurnSpeed,
                                                          heading=hdg, turn_in_place=True)
        execute_move(rotate_to_release)
        travel_controller.allow_reverse = True

    # travel only if not already at the target or closer to the ResourcePosition than approach target
    if distance_to_target > 10.0 or angle_to_target > 5.0:
        stats_manager.set_state('Transporting' if config.cont.ComponentCount > 0 else 'Moving')
        if distance_to_target > (travel_threshold + EPSILON):
            travel_to(travel_target, speed, comp.TurnSpeed, align_to_target=True)  # travel to approach loc
        if actuator_control and actuator_control.conf_load_extend == ActuatorControl.CONF_AT_APPROACH:
            actuator_control.extend(component.WorldPositionMatrix)
        move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)  # travel to target
        traffic_service.occupy_position(move_target.P, move_target.N)
        if actuator_control and actuator_control.conf_load_extend == ActuatorControl.CONF_AT_TARGET:
            actuator_control.extend(component.WorldPositionMatrix)

    elif comp.getProperty("Levels"):
        actuator_control.extend(component.WorldPositionMatrix)

    if manager.assistance == ActionManager.UNLOAD_ASSISTANT:
        # pick from assisted resource
        assist_wait_for_ready(task_action.AssistComponent)

    for pname in ('PickTime', 'GraspTime'):
        pick_time_p = task_action.TransportLink.getProperty(pname)
        if pick_time_p:
            break
    pick_time = pick_time_p.Value if pick_time_p else 0.0

    stats_manager.set_state('Picking')
    if assisted and not assist_requested:
        assist_request_assistance(task_action, load=True, to_mounted=True)
    if human_control and human_control.active and not current_tool:
        human_control.animate(component.Product.ProductType.Name + '_Pick', 'Default_Pick', lock=True)
    block_p = task_action.TransportLink.getProperty('BlockProcessPicking')
    if not assist_fasten_only:
        pick(component, pick_time, assisted, block_p.Value if block_p else False)
    if human_control:
        human_control.lock = False

    # Place_to_buffer
    if comp.getProperty(
            "Levels") and actuator_control and actuator_control.conf_load_extend == ActuatorControl.CONF_AT_TARGET:
        actuator_control.place_to_buffer(component)
        actuator_control.servo.moveJoint(1, 0.0)
        actuator_control.servo.moveJoint(0, actuator_control.servo.Joints[0].InitialValue)

    if not enable_pick_place_tolerance or (enable_pick_place_tolerance and distance_to_target > move_tolerance):
        # schedule pending move to reverse
        if not travel_controller.at_position(travel_target) and not travel_controller.pending_move_target:
            travel_controller.pending_move_target = travel_target
            travel_controller.pending_move_state = 'Picking'


def execute_place_action(manager, task_action):
    # type: (ActionManager, vcAction) -> None
    # debug('*** Executing ({}) action with properties {}'.format(task_action.Name, [(p.Name, p.Value) for p in task_action.Properties]), simtime=True)
    component = task_action.Component
    source_node = task_action.SourceNode
    destination_node, transport_target = get_transport_data(component, source_node, source=False)
    destination_container = destination_node.ComponentContainer
    offset_vec_p = task_action.TransportLink.getProperty('ResourcePlaceOffset')
    offset_vec = offset_vec_p.Value if offset_vec_p else vcVector.new()
    approach_vec_p = task_action.TransportLink.getProperty('PlaceApproach')
    approach_vec = approach_vec_p.Value if approach_vec_p else vcVector.new()
    approach_distance = approach_vec.length()
    product_target_wpm = transport_target.ProductPosition

    cart_comp = get_cart_for_product(component)
    if cart_comp:
        cart_offset = get_cart_offset(cart_comp)
    else:
        cart_offset = None

    move_target = transport_target.ResourcePosition  # type: vcMatrix
    move_target.translateRel(offset_vec.X, offset_vec.Y, offset_vec.Z)
    if cart_offset:
        move_target.translateRel(cart_offset.X, cart_offset.Y, cart_offset.Z)
    travel_target = vcMatrix.new(move_target)
    travel_target.translateRel(approach_vec.X, approach_vec.Y, approach_vec.Z)

    comp_wpm = travel_controller.interpolate(sim.SimTime)

    assisted = False
    assistant = False
    unload_assisting = None
    if manager.assistance == ActionManager.LOAD_ASSISTANT:
        # place on assisted resource
        # print comp.Name, 'assist: place on assisted resource'
        assistant = True
        assist_wait_for_ready(task_action.AssistComponent)
        destination_container = task_action.AssistComponent.getProperty('Assist::LoadContainer').Value
        product_target_wpm = task_action.AssistComponent.getProperty('Assist::LoadTarget').Value
        use_resourceloc_p = task_action.AssistComponent.getProperty('Assist::UseResourceLocation')
        if not use_resourceloc_p or not use_resourceloc_p.Value:
            move_target = assist_resolve_move_target(comp_wpm, product_target_wpm)
        else:
            move_target = comp_wpm  # keep pos
        travel_target = move_target
    elif manager.assistance == ActionManager.UNLOAD_ASSISTANT:
        # place on process
        assistant = True
        unload_assisting = task_action.AssistComponent
        # print comp.Name, 'assist: place on process'
        assist_frame = destination_node.Component.getFeature('AssistLocation')
        if assist_frame:
            move_target = destination_node.Component.WorldPositionMatrix * assist_frame.NodePositionMatrix
            travel_target = move_target
        else:
            use_resourceloc_p = task_action.AssistComponent.getProperty('Assist::UseResourceLocation')
            if not use_resourceloc_p or not use_resourceloc_p.Value:
                move_target = assist_resolve_move_target(comp_wpm, transport_target.ProductPosition)
            travel_target = move_target
    elif manager.assistance == ActionManager.UNLOAD_ASSISTED:
        assisted = True
        assist_requested = assist_request_assistance(task_action, load=False, to_mounted=False)

    if assistant:
        fasten_only_p = task_action.AssistComponent.getProperty('Assist::FastenOnly')
        assist_fasten_only = fasten_only_p.Value if fasten_only_p else False
    else:
        assist_fasten_only = False

    travel_threshold = approach_distance if approach_distance > 0.0 else 200.0
    distance_to_target = (comp_wpm.P - move_target.P).length()
    angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)

    enable_pick_place_tolerance_prop = comp.getProperty("Transport::EnablePickPlaceTolerance")
    move_tolerance_prop = comp.getProperty("Transport::MoveTolerance")

    if enable_pick_place_tolerance_prop and move_tolerance_prop:
        enable_pick_place_tolerance = enable_pick_place_tolerance_prop.Value
        move_tolerance = move_tolerance_prop.Value
    else:
        enable_pick_place_tolerance = False

    if enable_pick_place_tolerance and distance_to_target < move_tolerance:
        travel_controller.allow_reverse = False
        hdg = travel_controller.solve_move_heading(comp_wpm, travel_target)
        rotate_to_release = travel_controller.single_move(comp_wpm.P, comp.MoveSpeedApproach, comp.TurnSpeed,
                                                          heading=hdg, turn_in_place=True)
        execute_move(rotate_to_release)
        travel_controller.allow_reverse = True
        # schedule pending move to reverse

    # travel only if not already at the target or closer to the ResourcePosition than approach target
    elif distance_to_target > 10.0 or angle_to_target > 5.0:
        stats_manager.set_state('Transporting')
        if distance_to_target > (travel_threshold + EPSILON) and not assistant:
            travel_to(travel_target, comp.MoveSpeedLoaded, comp.TurnSpeed, align_to_target=True)
        if actuator_control and actuator_control.conf_unload_extend == ActuatorControl.CONF_AT_APPROACH:
            actuator_control.extend(product_target_wpm)
        move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
        traffic_service.occupy_position(move_target.P, move_target.N)
        if actuator_control and actuator_control.conf_unload_extend == ActuatorControl.CONF_AT_TARGET:
            if comp.getProperty("Levels"):
                actuator_control.pick_from_buffer(component)
            actuator_control.extend(product_target_wpm)
    elif comp.getProperty("Levels"):
        actuator_control.pick_from_buffer(component)
        actuator_control.extend(product_target_wpm)

        # wait for "WaitTransport" statement before placing product
    if task_action.TransportLink.WaitForNextTransport and manager.assistance in (
    ActionManager.UNASSISTED, ActionManager.UNLOAD_ASSISTANT, ActionManager.UNLOAD_ASSISTED):
        wait_for_transport(destination_node.ProcessExecutor)

    # is_last_action = manager.is_last_action()
    # if is_last_action:
    #   # TODO: check if charging is requested, don't prioritize or notify if so?
    #   # manager.prioritize_to_node(destination_node)
    #   if not assisted:
    #     manager.notify_tasks_completed(task_action) # NOTE: Call this before release product from hand i.e. grab to target!

    pnames = ('PlaceTime', 'ReleaseTime') if not assist_fasten_only else ('PickTime', 'GraspTime')
    for pname in pnames:
        place_time_p = task_action.TransportLink.getProperty(pname)
        if place_time_p:
            break
    place_time = place_time_p.Value if place_time_p else 0.0

    stats_manager.set_state('Placing')
    if assisted and not assist_requested:
        assist_request_assistance(task_action, load=False, to_mounted=True)
    if human_control and human_control.active and not current_tool:
        human_control.animate(component.Product.ProductType.Name + '_Place', 'Default_Place', lock=True)

    if comp.getProperty(
            "Levels") and actuator_control and actuator_control.conf_load_extend == ActuatorControl.CONF_AT_TARGET:
        actuator_control.servo.moveJoint(3, 0.0)
    place(component, destination_container, product_target_wpm, place_time, assisted, unload_assisting)

    if human_control:
        human_control.lock = False

    # Go home
    if comp.getProperty(
            "Levels") and actuator_control and actuator_control.conf_load_extend == ActuatorControl.CONF_AT_TARGET:
        actuator_control.servo.moveJoint(2, 0.0)
        actuator_control.servo.moveJoint(1, 0.0)
        actuator_control.servo.moveJoint(0, actuator_control.servo.Joints[0].InitialValue)

    if not enable_pick_place_tolerance or (enable_pick_place_tolerance and distance_to_target > move_tolerance):
        # schedule pending move to reverse
        if not travel_controller.at_position(travel_target) and not travel_controller.pending_move_target:
            travel_controller.pending_move_target = travel_target
            travel_controller.pending_move_state = 'Placing'


def execute_work_action(manager, task_action):
    # type: (ActionManager, vcAction) -> None

    debug('*** Executing ({}) action with properties {}'.format(task_action.Name,
                                                                [(p.Name, p.Value) for p in task_action.Properties]),
          simtime=True)
    tool_comp = task_action.ToolComponent
    if current_tool and task_action.Tool == 'No Tool' or task_action.Tool != 'Use Current' and tool_comp != current_tool:
        return_tool()

    if not current_tool and tool_comp:
        collect_tool(tool_comp)

    comp_wpm = travel_controller.interpolate(sim.SimTime)
    approach_vec = task_action.Approach
    approach_distance = approach_vec.length()
    move_target = task_action.ResourcePosition
    travel_target = vcMatrix.new(move_target)
    travel_target.translateRel(approach_vec.X, approach_vec.Y, approach_vec.Z)

    distance_to_target = (comp_wpm.P - move_target.P).length()
    angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)
    travel_threshold = approach_distance if approach_distance > 0.0 else 200.0

    # travel only if not already at the target or closer to the ResourcePosition than approach target
    if distance_to_target > 10.0 or angle_to_target > 5.0:
        stats_manager.set_state('Moving')
        if distance_to_target < travel_threshold:
            move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
        else:
            travel_to(travel_target, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)
            move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)

    stats_manager.set_state('Processing')
    if human_control and human_control.active:
        animating = False
        human_control.lock = True
        if current_tool:
            tool_anim = human_control.get_tool_anim_prefix(current_tool.Name)
            animating = human_control.animate(tool_anim + '_Work')
        if not animating and task_action.Executor:
            animating = human_control.animate(task_action.Executor.CurrentStatement.ParentRoutine.Name + '_Work')
        if not animating:
            human_control.animate('Default_Work')
    work_time = task_action.ProcessTime
    if work_time > 0:
        delay(work_time)
    else:
        warning('Invalid Work statement ProcessTime "{}". Processing skipped in zero time.'.format(work_time))
    manager.work_completed(task_action)
    if human_control:
        human_control.lock = False

    # schedule pending move to reverse
    if not travel_controller.at_position(travel_target) and not travel_controller.pending_move_target:
        travel_controller.pending_move_target = travel_target
        travel_controller.pending_move_state = 'Working'


def any_location_available(locations):
    return any([(x.Occupied < x.Capacity) for x in locations])


def on_idle_locations_released(arg=None):
    global idle_locations_available
    if any_location_available(idle_locations):
        idle_locations_available = True
        evaluateCondition()


def execute_idle(manager):
    global idle_locations_available, listened_occupied_properties, timeout_enabled
    stats_manager.set_state('Idle')
    execute_pending_move()
    if last_idle_chrg_loc:
        loc = last_idle_chrg_loc  # charging at idle
    else:
        if current_tool:
            if is_cart(current_tool):
                return_prop = current_tool.getProperty('ReturnTool')
                if return_prop and return_prop.Value:
                    return_tool()
            else:
                return_tool()
        loc = None
        while not loc:
            if config.power_manager_enabled and power_manager.charge_on_idle_p.Value:
                loc = reserve_charging_location()
            if not loc:
                loc = reserve_idle_location()
                if not loc:
                    warning('No idle locations available. Idling at place.')
                    idle_locations_available = False
                    # register to Occupied property's OnChanged event of each charging station and wait for trigger
                    for i in idle_locations:
                        prop = i.getProperty('Occupied')
                        prop.OnChanged = on_idle_locations_released
                        listened_occupied_properties.append(prop)
                    # wait until idle location released or new action available
                    condition(lambda: action_manager.actions_available() or idle_locations_available)
                    # if new action arrived, break the execution immediately
                    if action_manager.actions_available():
                        for prop in listened_occupied_properties:
                            prop.OnChanged = None
                        listened_occupied_properties = []
                        return

        # idling on path
        idling_on_path = loc.getProperty('IdlingOnPath')
        if idling_on_path and idling_on_path.Value:
            wp_interface_name = 'Waypoints'
            waypoints_iface = loc.findBehaviour(wp_interface_name)
            if not waypoints_iface:
                raise Exception(
                    'ERROR: Idle path component "{}" is missing "{}" interface!'.format(loc.Name, wp_interface_name))
            waypoints = waypoints_iface.ConnectedComponents
            if waypoints:
                current_index = waypoints[0].OrderIndex
                two_indices_found = False
                for wp in waypoints:
                    if wp.OrderIndex != current_index:
                        two_indices_found = True
                        break
            if not waypoints or not two_indices_found:
                print
                comp.Name, 'Error: Idle path component "{}" must have at least two waypoints connected with different indices.'.format(
                    loc.Name)
                return
            current_index -= 1
            waypoints.sort(key=lambda c: int(c.OrderIndex))
            while True:  # loop as long as interrupted
                for loc in waypoints:
                    if loc.OrderIndex == current_index:  # only visit same index once
                        continue
                    current_index = loc.OrderIndex
                    stats_manager.set_state('Idle')
                    # print 'travel to idle path index', loc.OrderIndex
                    pos_frame = loc.getFeature('ResourcePosition')  # type: vcFrameFeature
                    loc_wpm = loc.WorldPositionMatrix
                    target_wpm = loc_wpm * pos_frame.NodePositionMatrix if pos_frame else loc_wpm
                    completed = travel_to(target_wpm, comp.MoveSpeed, comp.TurnSpeed, align_to_target=False,
                                          interupt_on_action=True)
                    if not completed:  # new action arrived and interrupted
                        return

        # idling or charging at station
        else:
            stats_manager.set_state('Moving')
            loc_wpm = loc.WorldPositionMatrix
            pos_frame = loc.getFeature('ResourcePosition')  # type: vcFrameFeature
            target_wpm = loc_wpm * pos_frame.NodePositionMatrix if pos_frame else loc_wpm
            approach_frame = loc.getFeature('ApproachPosition')
            approach_mtx = loc_wpm * approach_frame.NodePositionMatrix if approach_frame else None

            if approach_mtx and (approach_mtx.P - target_wpm.P).length() > 1.0 and not travel_controller.at_position(
                    target_wpm):
                completed = travel_to(approach_mtx, comp.MoveSpeed, comp.TurnSpeed, align_to_target=False,
                                      interupt_on_action=True)
                if not completed:  # new action arrived and interrupted
                    return
                traffic_service.occupy_position(approach_mtx.P, approach_mtx.N)
                # Align to approach target
                align_move = travel_controller.single_move(approach_mtx.P, comp.MoveSpeedLoaded, comp.TurnSpeed,
                                                           approach_mtx.WPR.Z)
                travel_controller.activate_move(align_move)
                delay(align_move.duration)

                move_to(target_wpm, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
                traffic_service.occupy_position(target_wpm.P, target_wpm.N)
                travel_controller.pending_move_target = approach_mtx
                travel_controller.pending_move_state = ''
            else:
                completed = travel_to(target_wpm, comp.MoveSpeed, comp.TurnSpeed, align_to_target=False,
                                      interupt_on_action=True)
                if not completed:  # new action arrived and interrupted
                    return
                traffic_service.occupy_position(target_wpm.P, target_wpm.N)
                # Align to approach target
                align_move = travel_controller.single_move(target_wpm.P, comp.MoveSpeedLoaded, comp.TurnSpeed,
                                                           target_wpm.WPR.Z)
                travel_controller.activate_move(align_move)
                delay(align_move.duration)

            timeout_enabled = False
            if loc in charging_stations:
                stats_manager.set_state('Charging')
            else:
                stats_manager.set_state('Idle')


##### MISSION STEP EXECUTIONS

def execute_collect_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    # debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in mission_step.Properties]), simtime=True)
    task_actions = []
    while not task_actions:
        task_actions = manager.get_reserved_tasks(mission_step.action_index)
        if task_actions:
            while task_actions:
                for task_action in task_actions:
                    action_manager._evaluate_assistance(task_action)  # Added (Mission assistance)
                    action_manager.execute_pick_action(task_action)
                task_actions = manager.get_reserved_tasks(mission_step.action_index)
            return
        elif mission_step.optional_step:
            debug('No product found to collect. Skipping the optional step.', simtime=True)
            return  # skip step
        else:
            debug('No available products. Waiting for transport task.', simtime=True)
            stats_manager.set_state('Idle')
            condition(lambda: getTrigger())


def execute_collect_tool_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    # debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in mission_step.Properties]), simtime=True)
    collect_tool(mission_step.component)
    pass


def execute_deliver_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    # debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in mission_step.Properties]), simtime=True)
    task_actions = manager.get_reserved_tasks(mission_step.action_index)
    while task_actions:
        for task_action in task_actions:
            action_manager._evaluate_assistance(task_action)  # Added (Mission assistance)
            action_manager.execute_place_action(task_action)
        task_actions = manager.get_reserved_tasks(mission_step.action_index)
    return


def execute_deliver_tool_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    # debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in mission_step.Properties]), simtime=True)
    comp_wpm = travel_controller.interpolate(sim.SimTime)
    move_target = mission_step.position_matrix
    return_tool(False, 0, move_target, False)


def execute_wait_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    stats_manager.set_state('Break')
    delay(mission_step.wait_time)
    debug('Waiting time is over.', simtime=True)


def execute_charge_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    global timeout_enabled

    if not power_manager or not power_manager.enabled:
        debug('Charging is not possible; power manager must be enabled.'.format(comp.Name), simtime=True)
        return

    if power_manager.PCurrent.Value < (mission_step.to_charge_limit * power_manager.PCap.Value):
        loc = reserve_charging_location(mission_step.charging_locations)
        if loc:
            loc_wpm = loc.WorldPositionMatrix
            pos_frame = loc.getFeature('ResourcePosition')  # type: vcFrameFeature
            target_wpm = loc_wpm * pos_frame.NodePositionMatrix if pos_frame else loc_wpm
            approach_frame = loc.getFeature('ApproachPosition')
            approach_mtx = loc_wpm * approach_frame.NodePositionMatrix if approach_frame else None
            stats_manager.set_state('Moving')
            if approach_mtx and (approach_mtx.P - target_wpm.P).length() > 1.0 and not travel_controller.at_position(
                    target_wpm):
                travel_to(approach_mtx, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)
                traffic_service.occupy_position(approach_mtx.P, approach_mtx.N)
                move_to(target_wpm, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
                traffic_service.occupy_position(target_wpm.P, target_wpm.N)
                travel_controller.pending_move_target = approach_mtx
                travel_controller.pending_move_state = ''
            else:
                travel_to(target_wpm, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)
                traffic_service.occupy_position(target_wpm.P, target_wpm.N)
        else:
            warning('Couldn\'t reserve charging station {}'.format(charging_location.Name))
        stats_manager.set_state('Charging')
        timeout_enabled = False
        delay(power_manager.get_time_to_charging_limit(mission_step.charge_until_limit))  # forced charging time
        release_idle_or_charging_location()
        manager.available = True  # accept new tasks
        # action_manager.custom_task_completed(execute_charging)
    else:
        debug('charging not required.', simtime=True)


def execute_move_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    comp_wpm = travel_controller.interpolate(sim.SimTime)
    move_target = mission_step.position_matrix
    align_to_target = mission_step.align_to_target
    travel_target = vcMatrix.new(move_target)
    distance_to_target = (comp_wpm.P - move_target.P).length()
    angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)
    travel_threshold = 200.0

    # travel only if not already at the target or closer to the ResourcePosition than approach target
    if distance_to_target > 10.0 or angle_to_target > 5.0:
        stats_manager.set_state('Moving')
        if distance_to_target < travel_threshold:
            move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target)
        else:
            travel_to(travel_target, comp.MoveSpeed, comp.TurnSpeed, align_to_target)
            # move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
    stats_manager.set_state('Idle')


def execute_move_joint_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    stats_manager.set_state('Processing')
    servo_controller = mission_step.servo_controller
    if servo_controller:
        joint = servo_controller.findJoint(mission_step.joint_name)
        joint_index = servo_controller.Joints.index(joint)
    else:
        servos = comp.findBehavioursByType(VC_SERVOCONTROLLER)
        for servo in servos:
            joint = servo.findJoint(mission_step.joint_name)
            if joint:
                servo_controller = servo
                joint_index = servo.Joints.index(joint)
    servo_controller.setJointTarget(joint_index,
                                    mission_step.target_value)  # Bug fix 28375, the target must be declared first before setting the motion time
    if mission_step.motion_time != 0:
        servo_controller.setMotionTime(mission_step.motion_time)
    servo_controller.move()


def execute_run_robot_routine_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    stats_manager.set_state('Processing')
    mission_step.executor.callRoutine(mission_step.routine_name, mission_step.wait_execution)


def execute_wait_signal_step(manager, mission_step):
    global wait_signal_triggerers, triggerer
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)

    signal = mission_step.signal
    wait_signal_triggerers.append(signal)
    signal.OnValueChange = OnSignal

    stats_manager.set_state('Idle')
    if mission_step.wait_trigger:
        triggerCondition(
            lambda: triggerer.Name == mission_step.signal.Name and triggerer.Value == mission_step.signal_value,
            mission_step.timeout)
    else:
        condition(lambda: mission_step.signal.Value == mission_step.signal_value, mission_step.timeout)

    # if THIS_SCRIPT not in mission_step.signal.Connections:
    #   mission_step.signal.Connections += [THIS_SCRIPT]
    # if mission_step.wait_trigger:
    #   triggerCondition(lambda: getTrigger() == mission_step.signal and mission_step.signal.Value == mission_step.signal_value, mission_step.timeout)
    # else:
    #   condition(lambda: mission_step.signal.Value == mission_step.signal_value, mission_step.timeout)


def execute_send_signal_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    mission_step.signal.signal(mission_step.signal_value)


def execute_work_step(manager, mission_step):
    # type: (ActionManager, vcAction) -> None
    debug('*** Executing ({}) mission step with properties {}'.format(mission_step.Name, [(p.Name, p.Value) for p in
                                                                                          mission_step.Properties]),
          simtime=True)
    task_actions = []
    while not task_actions:
        task_actions = manager.get_reserved_tasks(mission_step.action_index)
        if task_actions:
            while task_actions:
                for task_action in task_actions:
                    action_manager.execute_work_action(task_action)
                task_actions = manager.get_reserved_tasks(mission_step.action_index)
            return
        elif mission_step.optional_step:
            debug('No work task requested in components {}. Skipping the optional step.'.format(
                list(set([n.Component.Name for n in mission_step.transport_nodes]))), simtime=True)
            return  # skip step
        else:
            debug('No work tasks requested in components {}. Waiting for a new work task.'.format(
                list(set([n.Component.Name for n in mission_step.transport_nodes]))), simtime=True)
            stats_manager.set_state('Idle')
            condition(lambda: getTrigger())


##### CUSTOM ACTIONS

def on_charging_stations_released(arg=None):
    global charging_stations_available
    if any_location_available(charging_stations):
        charging_stations_available = True
        evaluateCondition()


def execute_charging(manager):
    global charging_stations_available, timeout_enabled
    # NOTE: other actions are completed before charging (custom actions are executed last)
    loc = None
    while not loc:
        loc = reserve_charging_location()
        if not loc:
            charging_stations_available = False
            # register to Occupied property's OnChanged event of each charging station and wait for trigger
            # if trigger arrived, then break the execution immediately
            for cs in charging_stations:
                prop = cs.getProperty('Occupied')
                prop.OnChanged = on_charging_stations_released

            # move to idle location to wait to avoid traffic blockage
            stats_manager.set_state('Blocked')
            stay_put = 'stay in place' in power_manager.on_low_battery_p.Value
            if stay_put:
                idle_loc = None
            else:
                idle_loc = reserve_idle_location([x for x in idle_locations if
                                                  not x.getProperty('IdlingOnPath') or not x.getProperty(
                                                      'IdlingOnPath').Value])
            # move to idle
            if idle_loc:
                # stats_manager.set_state('Moving')
                warning(
                    'Battery low level threshold reached. No charging stations available. Moving to idle station "{}"'.format(
                        idle_loc.Name))
                loc_wpm = idle_loc.WorldPositionMatrix
                pos_frame = idle_loc.getFeature('ResourcePosition')  # type: vcFrameFeature
                target_wpm = loc_wpm * pos_frame.NodePositionMatrix if pos_frame else loc_wpm
                completed = travel_to(target_wpm, comp.MoveSpeed, comp.TurnSpeed, align_to_target=False,
                                      interrupt_on_charging_stations_released=True)  # interrupt motion if station released
                if completed:
                    traffic_service.occupy_position(target_wpm.P, target_wpm.N)
                    align_move = travel_controller.single_move(target_wpm.P, comp.MoveSpeedLoaded, comp.TurnSpeed,
                                                               target_wpm.WPR.Z)
                    travel_controller.activate_move(align_move)
                    delay(align_move.duration)
            else:
                warning('Battery low level threshold reached. No charging stations available. Going to standby mode.')
            condition(lambda: charging_stations_available)
            if idle_loc:
                release_idle_or_charging_location()
        else:
            # unregister the Capacity property change event of each charging stations
            for cs in charging_stations:
                prop = cs.getProperty('Occupied')
                # prop.OnChanged = None

    loc_wpm = loc.WorldPositionMatrix
    pos_frame = loc.getFeature('ResourcePosition')  # type: vcFrameFeature
    target_wpm = loc_wpm * pos_frame.NodePositionMatrix if pos_frame else loc_wpm
    approach_frame = loc.getFeature('ApproachPosition')
    approach_mtx = loc_wpm * approach_frame.NodePositionMatrix if approach_frame else None

    if approach_mtx and (approach_mtx.P - target_wpm.P).length() > 1.0 and not travel_controller.at_position(
            target_wpm):
        travel_to(approach_mtx, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)
        traffic_service.occupy_position(approach_mtx.P, approach_mtx.N)

        move_to(target_wpm, comp.MoveSpeedApproach, comp.TurnSpeed, align_to_target=True)
        traffic_service.occupy_position(target_wpm.P, target_wpm.N)

        # hdg = travel_controller.solve_move_heading(travel_controller.position_matrix, approach_mtx)
        # reverse_move = travel_controller.single_move(approach_mtx.P, comp.MoveSpeedApproach, comp.TurnSpeed, heading=hdg, turn_in_place=True)
        # travel_controller.pending_move = reverse_move
        travel_controller.pending_move_target = approach_mtx
        travel_controller.pending_move_state = ''
    else:
        travel_to(target_wpm, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)
        traffic_service.occupy_position(target_wpm.P, target_wpm.N)

    stats_manager.set_state('Charging')
    timeout_enabled = False
    delay(power_manager.get_time_to_charging_limit())  # forced charging time
    manager.available = True  # accept new tasks
    action_manager.custom_task_completed(execute_charging)


def execute_break(manager):
    global timeout_enabled
    timeout_enabled = False
    stats_manager.set_state('Break')
    loc = reserve_idle_location()
    if loc:
        travel_to(loc.WorldPositionMatrix, comp.MoveSpeed, comp.TurnSpeed, align_to_target=True)


#### OTHERS
def wait_for_transport(process_executor):
    process_executor.OnPreExecuteStatement = activate_trigger
    while process_executor.CurrentStatement.Type != VC_STATEMENT_WAITTRANSPORT:
        stats_manager.set_state('Blocked')
        triggerCondition(lambda: True)
        delay(0.001)
    process_executor.OnPreExecuteStatement = None


def on_availability_changed(avail_p):
    if not action_manager.connected_controller:
        return
    if avail_p.Value:
        # if availability changed to True, cancel any scheduled breaks (if any)
        action_manager.set_availability(True)
        action_manager.cancel_custom_action(execute_break)
        action_manager.custom_task_completed(execute_break)
    else:
        # if availability changed to False, schedule break if not already scheduled
        action_manager.set_availability(False)
        if execute_break not in action_manager.custom_task_actions:
            action_manager.schedule_custom_action(execute_break)
        if stats_manager and stats_manager.statistics.State != 'Blocked:':
            evaluateCondition()  # excite OnRun


def execute_pending_move():
    if not travel_controller.pending_move_target:
        return
    elif travel_controller.at_position(travel_controller.pending_move_target):
        return
    else:
        stats_manager.set_state('Transporting' if config.cont.ComponentCount > 0 else 'Moving')
        if actuator_control:
            retract_target = False
            retract_exit = False
            if travel_controller.pending_move_state == 'Picking':
                retract_target = actuator_control.conf_load_retract == ActuatorControl.CONF_AT_TARGET
                retract_exit = actuator_control.conf_load_retract == ActuatorControl.CONF_AT_EXIT
            elif travel_controller.pending_move_state == 'Placing':
                retract_target = actuator_control.conf_unload_retract == ActuatorControl.CONF_AT_TARGET
                retract_exit = actuator_control.conf_unload_retract == ActuatorControl.CONF_AT_EXIT
            if retract_target: actuator_control.retract()

        hdg = travel_controller.solve_move_heading(travel_controller.position_matrix,
                                                   travel_controller.pending_move_target)
        pending_move = travel_controller.single_move(travel_controller.pending_move_target.P, comp.MoveSpeedApproach,
                                                     comp.TurnSpeed, heading=hdg, turn_in_place=True)
        execute_move(pending_move)
        traffic_service.occupy_position(travel_controller.pending_move_target.P,
                                        travel_controller.pending_move_target.N)
        if actuator_control and retract_exit:
            actuator_control.retract()
        stats_manager.set_state('Idle')
        # travel_controller.pending_move = None
        travel_controller.pending_move_target = None
        travel_controller.pending_move_state = ''


def move_to(final_mtx, velocity, turn_velocity, align_to_target=False):
    if travel_controller.at_position(final_mtx):
        return  # already at the location

    hdg = travel_controller.solve_move_heading(travel_controller.position_matrix, final_mtx, align_to_target)
    move = travel_controller.single_move(final_mtx.P, velocity, turn_velocity, heading=hdg, turn_in_place=True)

    execute_move(move)
    # travel_controller.activate_move(move)
    # delay(move.duration)

    if align_to_target and not travel_controller.at_position(
            final_mtx) and travel_controller.carriage_type != TravelController.CONFIG_CARRIAGE_OMNI:
        align_hdg = travel_controller.solve_align_heading(final_mtx)
        align_move = travel_controller.single_move(final_mtx.P, velocity, turn_velocity, align_hdg)
        travel_controller.activate_move(align_move)
        delay(align_move.duration)

    OnSimulationUpdate(sim.SimTime)


def execute_move(move):
    # type: (Move) -> None
    travel_controller.activate_move(move)

    if collision_mgr.state == CollisionManager.STATE_DISABLED:
        travel_controller.activate_move(move)
        delay(move.duration)
        OnSimulationUpdate(sim.SimTime)
        return

    move_state = stats_manager.statistics.State
    hdg_mtx = travel_controller.get_hdg_matrix()
    ttt = move.get_time_to_target()
    while ttt > 0:
        travel_controller.interpolate(sim.SimTime)
        hdg_mtx.P = travel_controller.position_matrix.P
        collision_mgr.evaluate(hdg_mtx)

        avoid_res = collision_mgr.stop_detector.result
        stop = False
        sample_interval = collision_mgr.sampling_interval
        if collision_mgr.state == CollisionManager.STATE_STOPPED:
            stop = not collision_mgr.allow_ignoring or (
                        avoid_res.avoiding_p.Value != comp and travel_controller.current_area and not travel_controller.current_area.in_queue(
                    avoid_res.component))
        if stop:
            move.pause()  # other not avoiding this resource (not head-to-head) or other queuing to the same area
            stats_manager.set_state('Blocked')
        else:
            if collision_mgr.state == CollisionManager.STATE_FREE:
                sample_interval = collision_mgr.time_to_next_potential_hit(hdg_mtx)
            move.resume()
            stats_manager.set_state(move_state)
        ttt = move.get_time_to_target()
        delay(ttt if ttt < sample_interval else sample_interval)
        collision_mgr.reset()
    OnSimulationUpdate(sim.SimTime)


def travel_to(final_mtx, velocity, turn_velocity, align_to_target=False, interupt_on_action=False,
              interrupt_on_charging_stations_released=False):
    global charging_stations_available
    ''' Returns True if destination reached without action interupt '''
    if travel_controller.at_position(final_mtx):
        return  # already at the location

    current_pos_mtx = travel_controller.position_matrix
    path = travel_controller.get_path(current_pos_mtx.P, final_mtx.P, velocity)
    travel_controller.activate_path(path)

    triggers = (traffic_service.collision_signal, traffic_service.update_signal)
    scheduler = travel_scheduler
    scheduler.reset()
    scheduler.update()
    travel_state = stats_manager.statistics.State

    # print comp.Name, 'STARTING TRAVEL...'
    while scheduler.time_to_target > 0:
        # print comp.Name, 'next_event_time', scheduler.next_event_time, 'Type:', scheduler.next_event
        if scheduler.next_event_time > 0:
            if interupt_on_action:
                collision_event = triggerCondition(
                    lambda: getTrigger() in triggers or action_manager.actions_available(), scheduler.next_event_time)
                if action_manager.actions_available():
                    travel_controller.interpolate(sim.SimTime)
                    collision_mgr.reset()
                    path.update_stats()
                    OnSimulationUpdate(sim.SimTime)
                    travel_controller.deactivate()
                    return False
            elif interrupt_on_charging_stations_released:
                collision_event = condition(lambda: charging_stations_available, scheduler.next_event_time)
                if charging_stations_available:
                    travel_controller.interpolate(sim.SimTime)
                    collision_mgr.reset()
                    path.update_stats()
                    OnSimulationUpdate(sim.SimTime)
                    travel_controller.deactivate()
                    return False
            else:
                collision_event = triggerCondition(lambda: getTrigger() in triggers, scheduler.next_event_time)

            if collision_event and not scheduler.in_collision:  # or traffic_service.update_available():
                # print comp.Name, 'RECEIVED COLLISION EVENT', sim.SimTime
                traffic_service.update_info()
                scheduler.next_event = TravelScheduler.EVENT_COLLISION

        next_event = scheduler.next_event
        travel_controller.interpolate(sim.SimTime)

        if next_event == TravelScheduler.EVENT_AREA_CROSSING and travel_controller.current_area != path.next_area:
            fetch_traffic = True
            next_area = path.next_area
            # print scheduler.area_group_entry, sim.SimTime

            wait_for_capacity = scheduler.area_group_entry and not next_area.group.has_capacity(
                path) or next_area and not next_area.has_capacity()
            if wait_for_capacity:
                # wait for capacity
                skip_queue = False
                if collision_mgr.state == CollisionManager.STATE_IGNORING and travel_controller.current_area:
                    # exchange areas with other resource and skip queue to next area
                    queue = travel_controller.current_area.queue_p.Value
                    skip_queue = collision_mgr.avoiding_p.Value in queue

                if not skip_queue:
                    global manual_pause
                    manual_pause = False

                    path.pause()
                    scheduler.in_collision = False
                    collision_mgr.reset()
                    # traffic_service.avoiding_p.Value = None
                    traffic_service.update_status()  # in case speed changed/stopped

                    stats_manager.set_state('Blocked')

                    next_area.place_on_queue(comp)
                    if scheduler.area_group_entry:
                        next_area.group.place_on_queue(comp)
                        for area in next_area.group.areas:
                            area.used_capacity_p.OnChanged = activate_trigger
                            area.capacity_p.OnChanged = activate_trigger  # Bug fix 30559, Resource was not moving to area when pathway's capacity set to non-zero
                        triggerCondition(lambda: next_area.group.has_capacity(path) and next_area.group.next_in_queue(
                            comp) and next_area.next_in_queue(comp))
                        for area in next_area.group.areas:
                            area.used_capacity_p.OnChanged = None
                            area.capacity_p.OnChanged = None
                        next_area.group.remove_from_queue()
                    else:
                        next_area.capacity_p.OnChanged = activate_trigger
                        next_area.used_capacity_p.OnChanged = activate_trigger  # Bug fix 30559, Resource was not moving to area when pathway's capacity set to non-zero
                        # triggerCondition(lambda: next_area.has_capacity() and next_area.next_in_queue(comp)) # this was only letting one resource in, although the pathway capacity was increased by multiple
                        triggerCondition(lambda: next_area.has_capacity() and next_area.index_in_queue(comp) < (
                                    next_area.capacity_p.Value - next_area.used_capacity_p.Value))
                        next_area.used_capacity_p.OnChanged = None
                        next_area.capacity_p.OnChanged = None

                # enter the next area
                next_area.used_capacity_p.Value += 1
                next_area.remove_from_queue()
                path.resume()
                path.set_speeds(path.velocity_setting, path.velocity_setting)
                stats_manager.set_state(travel_state)
            elif next_area:
                # enter the next area
                path.next_area.used_capacity_p.Value += 1
                fetch_traffic = False

            # leave previous area
            if travel_controller.current_area:
                travel_controller.current_area.used_capacity_p.Value -= 1
            travel_controller.current_area = path.next_area

        elif next_event == TravelScheduler.EVENT_COLLISION:
            hdg_mtx = travel_controller.get_hdg_matrix()
            hdg_mtx.P = travel_controller.position_matrix.P
            state_change = collision_mgr.evaluate(hdg_mtx, travel_controller.current_area)
            scheduler.in_collision = collision_mgr.state > CollisionManager.STATE_FREE

            if collision_mgr.speed_setting < 0 or collision_mgr.speed_setting > path.velocity_setting:
                new_speed = path.velocity_setting
            else:
                new_speed = collision_mgr.speed_setting

            if path.current_velocity_setting != new_speed:
                # print comp.Name, 'SPEED CHANGE:', path.current_velocity_setting, '->', new_speed
                path.set_speeds(new_speed, new_speed)
                traffic_service.update_status()  # in case speed changed/stopped
                if new_speed <= 0:
                    stats_manager.set_state('Blocked')
                else:
                    stats_manager.set_state(travel_state)

            other_resource = collision_mgr.avoiding_p.Value
            if not other_resource:
                other_resource = traffic_service.other_resource

            if state_change and scheduler.in_collision:
                if not other_resource.getProperty('Nav::Scanning').Value and not other_resource.getProperty(
                        'Nav::Paused').Value:
                    # notify other (force scanning)
                    for res in (r for r in collision_mgr.near_resources if
                                not r.scanning_p.Value and not r.paused_p.Value):
                        res.collision_signal.signal(comp)
                if collision_mgr.state == CollisionManager.STATE_STOPPED and path.active_move:
                    path.active_move.stats_collision_count += 1
            fetch_traffic = not scheduler.in_collision
        else:
            fetch_traffic = False
        scheduler.update(fetch_traffic=fetch_traffic)

    # in case destination reached while in collision, clear collision
    collision_mgr.reset()

    path.update_stats()
    if align_to_target and not travel_controller.at_position(final_mtx):
        traffic_service.occupy_position(final_mtx.P, final_mtx.N)
        align_hdg = travel_controller.solve_align_heading(final_mtx)
        align_move = travel_controller.single_move(final_mtx.P, velocity, turn_velocity, align_hdg, True,
                                                   2)  # Added (KIVA) - Add last argument
        travel_controller.activate_move(align_move)
        delay(align_move.duration)
    OnSimulationUpdate(sim.SimTime)
    return True


def pick(component, pick_time, assisted=False, block_process=False):
    global robo

    if pick_time < 0.0:
        pick_time = 0.0
    cont = config.cont
    prod_type_name = component.Product.ProductType.Name
    component.update()

    prod_target_wpm = None
    pattern = None
    if current_tool:
        loc_node = find_node(current_tool, 'TransportNode')
        if not loc_node:
            loc_node = current_tool
        if is_cart(current_tool):
            # Cart
            pattern = get_cart_pattern(current_tool, prod_type_name)
            conts = loc_node.findBehavioursByType(VC_CONTAINER)
            if conts:
                cont = conts[0]
        else:
            # Other tool
            for frame_name in (prod_type_name + '_location', 'DEFAULT_location'):
                prod_target_wpm = return_feature_world_location(loc_node, frame_name)
                if prod_target_wpm:
                    pattern = pattern_manager.default_pattern  # only a logical placeholder, slot locations not used
                    break
    else:
        # No tool
        pattern = pattern_manager.get_pattern(prod_type_name)
    if not pattern:
        pattern = pattern_manager.get_pattern(prod_type_name)
    slot = pattern.get_free_slot()
    slot.component = component  # reserve slot
    if not prod_target_wpm:
        # Without tool or tool has pattern
        prod_target_wpm = pattern.get_world_pos_matrix(slot)

    offset_mtx = component.Product.ProductType.OriginFrame
    if offset_mtx != IDENTITY_MTX:
        offset_mtx.invert()
        prod_target_wpm *= offset_mtx

    if not robo and pattern.keep_orientation:
        prod_target_wpm.WPR = component.WorldPositionMatrix.WPR

    if assisted:
        action_manager.assist_load_target_p.Value = prod_target_wpm
        action_manager.assist_load_cont_p.Value = cont
        cont.OnTransition = activate_trigger
        ready_p = action_manager.assist_ready_p
        ready_p.Value = True
        triggerCondition(lambda: component in cont.Components)
        cont.OnTransition = None
        ready_p.Value = False
        return

    if comp.SimulationLevel == VC_SIMULATION_DETAILED and sim.SimSpeed < pick_time * 10:
        if robo:
            # toolOK = doEoatChange(robo, robot_tool_iface, *tooldata)
            robo.Configuration = robot_config_prop.Value
            robo.jointMoveToMtx(component.WorldPositionMatrix, Tz=component.BoundDiagonal.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(component.WorldPositionMatrix, Tz=component.BoundDiagonal.Z * 2.0, Rx=180.0)
            robo.graspComponent(component)
            robo.jointMoveToMtx(component.WorldPositionMatrix, Tz=component.BoundDiagonal.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(prod_target_wpm, Tz=component.BoundDiagonal.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(prod_target_wpm, Tz=component.BoundDiagonal.Z * 2.0, Rx=180.0)
            cont.grab(component)
            robo.jointMoveToMtx(prod_target_wpm, Tz=component.BoundDiagonal.Z * 2.0 + 50.0, Rx=180.0)
            robo.driveJoints(*robot_home_joint_vals)
        else:
            # Added (FABRIK)
            if looks:
                comp.Looks = "Adam"
                comp.Target = component
            obj_itp = ObjectInterpolator(component, prod_target_wpm, cont, pick_time, grab_to_world=not block_process)
            obj_interp_manager.add(obj_itp)
            obj_interp_manager.wait_to_complete(obj_itp)
            if block_process:
                world_container.grab(component)
                component.PositionMatrix = prod_target_wpm
    else:
        if not block_process:
            world_container.grab(component)
        delay(pick_time)
        if block_process:
            world_container.grab(component)
        component.PositionMatrix = prod_target_wpm

    # Added (KIVA)
    if transport_raycast_show_raycast:
        travel_controller.detect_lift_distance()

        if travel_controller.detected_signal.Value:
            travel_controller.move_lift(travel_controller.range_signal.Value, False)

    cont.grab(component)

    # Added (KIVA)
    if transport_raycast_show_raycast:
        if travel_controller.detected_signal.Value:
            travel_controller.product_orientation_offset = travel_controller.position_matrix.WPR.Z
            travel_controller.move_lift(
                travel_controller.product_lift.Value + travel_controller.extra_lift_height.Value, True)

    # Added (FABRIK)
    if looks:
        comp.Looks = "Otto"
        comp.Target = None


def place(component, dest_container, dest_prod_wpm, place_time, assisted=False, unload_assisting=None):
    global robo
    if place_time < 0.0:
        place_time = 0.0

    slot = pattern_manager.find_slot(component)
    if slot:
        slot.component = None  # release slot

    offset_mtx = component.Product.ProductType.OriginFrame
    if offset_mtx != IDENTITY_MTX:
        offset_mtx.invert()
        dest_prod_wpm *= offset_mtx

    component.update()
    if not robo and slot and slot.pattern.keep_orientation:
        dest_prod_wpm.WPR = component.WorldPositionMatrix.WPR

    if assisted:
        dest_container.OnTransition = activate_trigger
        action_manager.pre_process_trigger(add_capacity=0)
        ready_p = action_manager.assist_ready_p
        ready_p.Value = True
        triggerCondition(lambda: component in dest_container.Components)
        dest_container.OnTransition = None
        ready_p.Value = False
        action_manager.post_process_trigger()
        return

    if comp.SimulationLevel == VC_SIMULATION_DETAILED and sim.SimSpeed < place_time * 10:
        if robo:
            pick_wpm = component.WorldPositionMatrix
            comp_bd = component.BoundDiagonal
            robo.Configuration = robot_config_prop.Value
            robo.jointMoveToMtx(pick_wpm, Tz=comp_bd.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(pick_wpm, Tz=comp_bd.Z * 2.0, Rx=180.0)
            robo.graspComponent(component)
            robo.jointMoveToMtx(pick_wpm, Tz=comp_bd.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(dest_prod_wpm, Tz=comp_bd.Z * 2.0 + 50.0, Rx=180.0)
            robo.jointMoveToMtx(dest_prod_wpm, Tz=comp_bd.Z * 2.0, Rx=180.0)
            action_manager.pre_process_trigger(unload_assisting)
            dest_container.grab(component)
            action_manager.post_process_trigger()
            robo.jointMoveToMtx(dest_prod_wpm, Tz=comp_bd.Z * 2.0 + 50.0, Rx=180.0)
            robo.driveJoints(*robot_home_joint_vals)
            return
        else:
            # Added (FABRIK)
            if looks:
                comp.Looks = "Adam"
                comp.Target = component
            # Added (KIVA)
            if transport_raycast_show_raycast:
                delay(place_time)
            else:
                obj_itp = ObjectInterpolator(component, dest_prod_wpm, dest_container, place_time, grab_to_world=False)
                obj_interp_manager.add(obj_itp)
                obj_interp_manager.wait_to_complete(obj_itp)
    else:
        delay(place_time)

    # Added (KIVA)
    if transport_raycast_show_raycast:
        if travel_controller.detected_signal.Value:
            travel_controller.move_lift(
                travel_controller.product_lift.Value - travel_controller.extra_lift_height.Value, True)

    world_container.grab(component)
    component.PositionMatrix = dest_prod_wpm

    action_manager.pre_process_trigger(unload_assisting)
    dest_container.grab(component)

    # Added (KIVA)
    if transport_raycast_show_raycast:
        if travel_controller.detected_signal.Value:
            travel_controller.move_lift(0.0, False)

            # Added (FABRIK)
    if looks:
        comp.Looks = "Otto"
        comp.Target = None

    action_manager.post_process_trigger()


def collect_tool(tool_comp, move_speed=0):
    # type: (vcComponent) -> None
    global current_tool

    if not tool_comp:
        return

    if tool_comp in comp.ChildComponents:
        # Already attached
        current_tool = tool_comp
        return

    if move_speed <= 0:
        move_speed = comp.MoveSpeed
    tool_comp.update()

    # Calculate move target for pickup
    move_target = None
    parent_iface = None
    child_iface = None
    if is_cart(tool_comp):
        parent_iface = get_cart_interface()
        first_cart = tool_comp
        child_iface = first_cart.findBehaviour('Parent')
        while child_iface and child_iface.IsConnected:
            first_cart = child_iface.ConnectedComponent
            child_iface = first_cart.findBehaviour('Parent')
        if parent_iface and child_iface:
            parent_offset = parent_iface.Parent.PositionMatrix
            parent_offset.invert()
            child_frame = child_iface.Sections[0].Frame
            child_offset = child_frame.NodePositionMatrix
            move_target = first_cart.WorldPositionMatrix * child_offset * parent_offset

    if not move_target:
        resframe = find_feature(tool_comp, 'ResourceLocation')
        if resframe:
            move_target = tool_comp.WorldPositionMatrix * resframe.NodePositionMatrix
        else:
            move_target = tool_comp.WorldPositionMatrix

    approach_vec = tool_comp.getProperty('PM::ToolChangeApproach').Value
    approach_distance = approach_vec.length()
    travel_target = vcMatrix.new(move_target)
    travel_target.translateRel(approach_vec.X, approach_vec.Y, approach_vec.Z)

    comp_wpm = travel_controller.interpolate(sim.SimTime)
    travel_threshold = approach_distance if approach_distance > 0.0 else 200.0
    distance_to_target = (comp_wpm.P - move_target.P).length()
    angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)
    stats_manager.set_state('CollectingTool')

    # travel only if not already at the target or closer to the ResourcePosition than approach target
    if distance_to_target > 10.0 or angle_to_target > 5.0:
        if distance_to_target > (travel_threshold + EPSILON):
            travel_to(travel_target, move_speed, comp.TurnSpeed)
        move_to(move_target, move_speed, comp.TurnSpeed, align_to_target=True)

    OnSimulationUpdate(sim.SimTime)

    if parent_iface and child_iface:
        parent_iface.connect(child_iface)
    else:
        tool_container = config.tool_cont  # type: vcComponentContainer
        tool_container.grab(tool_comp)

    current_tool = tool_comp
    tool_change_delay = tool_comp.getProperty('PM::ToolChangeDelay')

    delay(tool_change_delay.Value)

    if approach_distance > 0.0:
        move_to(travel_target, move_speed, comp.TurnSpeed)  # reverse=True


def return_tool(force=False, move_speed=0, move_target=None, set_init_loc=True):
    # type: (bool, float, vcMatrix, bool) -> None
    global current_tool

    if not current_tool:
        return

    parent_iface = None
    child_iface = None
    align_at_approach = False
    align_at_target = True
    if is_cart(current_tool):
        parent_iface = get_cart_interface(False)
        if parent_iface and parent_iface.IsConnected:
            child_iface = parent_iface.ConnectedSection.ConnectedToSection.Interface

    if move_speed <= 0:
        move_speed = comp.MoveSpeed

    return_tool_p = current_tool.getProperty('ReturnTool')
    if force or return_tool_p and return_tool_p.Value or move_target:
        tool_initial_loc = current_tool.getProperty('PM::InitialLoc').Value
        approach_vec = current_tool.getProperty('PM::ToolChangeApproach').Value
        approach_distance = approach_vec.length()

        tool_change_delay = current_tool.getProperty('PM::ToolChangeDelay')

        if parent_iface and child_iface:
            align_at_approach = True
            align_at_target = False

            parent_offset = parent_iface.Parent.PositionMatrix
            parent_offset.invert()
            child_frame = child_iface.Sections[0].Frame
            child_offset = child_frame.NodePositionMatrix
            if move_target:
                move_target = move_target * child_offset * parent_offset
            else:
                move_target = tool_initial_loc * child_offset * parent_offset
        elif not move_target:
            resframe = find_feature(current_tool, 'ResourceLocation')
            if resframe:
                move_target = resframe.NodePositionMatrix * tool_initial_loc
            else:
                move_target = tool_initial_loc

        travel_target = vcMatrix.new(move_target)
        travel_target.translateRel(approach_vec.X, approach_vec.Y, approach_vec.Z)

        comp_wpm = travel_controller.interpolate(sim.SimTime)
        travel_threshold = approach_distance if approach_distance > 0.0 else 200.0
        distance_to_target = (comp_wpm.P - move_target.P).length()
        angle_to_target = abs(comp_wpm.WPR.Z - move_target.WPR.Z)
        stats_manager.set_state('ReturningTool')

        # travel only if not already at the target or closer to the ResourcePosition than approach target
        if distance_to_target > 10.0 or angle_to_target > 5.0:
            if distance_to_target > (travel_threshold + EPSILON):
                travel_to(travel_target, move_speed, comp.TurnSpeed, align_at_approach)
            else:
                align_at_target = True
            move_to(move_target, comp.MoveSpeedApproach, comp.TurnSpeed, align_at_target)

        wpm = current_tool.WorldPositionMatrix
        delay(tool_change_delay)

        if parent_iface and child_iface:
            parent_iface.disconnect()
        else:
            sim.World.attach(current_tool, False)
        if set_init_loc:
            current_tool.PositionMatrix = tool_initial_loc
        else:
            current_tool.PositionMatrix = wpm
        current_tool.update()

        move_to(travel_target, comp.MoveSpeedApproach, comp.TurnSpeed)  # reverse=True
    else:
        if parent_iface and child_iface:
            parent_iface.disconnect()
        else:
            current_tool.update()
            wpm = current_tool.WorldPositionMatrix
            sim.World.attach(current_tool, False)
            current_tool.PositionMatrix = wpm
            current_tool.update()

    current_tool.getProperty('PM::ReservedTo').Value = None
    current_tool = None


def get_transport_data(component, source_node, source=True):
    # type: (vcComponent, vcTransportNode, bool) -> Tuple[vcTransportNode, vcTransportTarget]
    product = component.Product
    solution = product.TransportSolution

    if len(solution.Links) == 1:
        node = solution.Source if source else solution.Destination
    elif source:
        node = solution.getNextLink(source_node).Source
    else:
        node = solution.getNextLink(source_node).Destination

    return node, node.getTransportTarget(product)


def stats_dump_request(prop):
    if travel_controller and prop.Value:
        # print comp.Name, 'Dumping..'
        travel_controller.dump_stats()
    prop.Value = False


def toggle_power_prop_visibility(toggle_prop):
    if toggle_prop.Name.startswith('Power::'):
        for prop in (p for p in comp.Properties if p.Name.startswith('Power::') and p.Name != toggle_prop.Name):
            prop.IsVisible = toggle_prop.Value


def on_charger_priority_changed(prop):
    local_priority_p = comp.getProperty('Power::ChargerPriorityOrder')
    if local_priority_p:
        local_priority_p.IsVisible = (prop.Value == "Local Priority Order")
    if 'Global Priority Order' in prop.Value:
        tc = comp.findBehaviour('PMResource').ConnectedComponent
        gp = tc.getProperty('ChargerPriorityOrder') if tc else None
        if gp: gp.IsVisible = True


def is_valid_charging_location(loc_comp):
    charger_prop = loc_comp.getProperty('Charger')
    if loc_comp.findBehaviour('ChargingLocation') or (charger_prop and charger_prop.Value):
        return True
    return False


def is_valid_idle_location(loc_comp):
    idle_prop = loc_comp.getProperty('AllowIdling')
    if loc_comp.findBehaviour('IdleLocation') or (idle_prop and idle_prop.Value):
        return True
    return False


enable_priority_order_calls = True


def charger_priority_order_changed(prop):
    global enable_priority_order_calls
    if not enable_priority_order_calls:
        return
    tc = comp.findBehaviour('PMResource').ConnectedComponent
    if not tc:
        warning('Controller not connected. Cannot prioritize charging locations.')
        return
    selected_components = prop.Value
    connected = []
    for loc_comp in selected_components:
        if is_valid_charging_location(loc_comp):
            # connect selected location component, if not connected already
            idle_interface = tc.findBehaviour('Idle/ChargingLocations')
            if loc_comp not in idle_interface.ConnectedComponents:
                for other_ife in loc_comp.findBehavioursByType(VC_ONETOMANYINTERFACE):
                    if not idle_interface.canConnect(other_ife):
                        continue
                    if idle_interface.connect(other_ife):
                        connected.append(other_ife.Component)
        else:
            warning('"{}" is not valid component for {}'.format(loc_comp.Name, prop.Name))
            selected_components.remove(loc_comp)
    enable_priority_order_calls = False
    prop.Value = selected_components
    enable_priority_order_calls = True
    if connected:
        info('Connected to {} new interface(s): {}. (connected total: {})'.format(len(connected), ', '.join(
            '"' + c.Name + '"' for c in connected), len(idle_interface.ConnectedComponents)))


local_priority_p = comp.getProperty('Power::ChargerPriorityOrder')
local_priority_p.OnChanged = charger_priority_order_changed


def initialize_patterns(manager):
    # type: (PatternManager) -> None
    ref_node = comp.findNode('TransportNode')
    if not ref_node:
        ref_node = comp
    for prod_name in location_ui_type_drop_down_p.StepValues:
        if prod_name == 'Default <Any>':
            prod_name = 'DEFAULT'
        if prod_name == 'Add New...':
            continue
        ref_pos_frame = find_feature(comp, prod_name + '_location')
        pattern_p = comp.getProperty('Transport::Pattern_' + prod_name)
        pattern_step_p = comp.getProperty('Transport::PatternStep_' + prod_name)
        keep_ori_p = comp.getProperty('Transport::KeepOrientation_' + prod_name)
        ptrn = Pattern(prod_name, ref_node, ref_pos_frame.NodePositionMatrix, pattern_p.Value, pattern_step_p.Value,
                       keep_ori_p.Value)
        manager.add_pattern(ptrn)
        if prod_name == 'DEFAULT':
            manager.set_default(ptrn)


def assist_request_assistance(task_action, load=True, to_mounted=False):
    # type: (vcComponent, bool) -> bool
    prop_name = 'LoadAssist' if load else 'UnloadAssist'
    # print comp.Name, 'REQUEST:', prop_name
    assistant_comp = task_action.TransportLink.getProperty(prop_name).Value
    mounted_tc = assistant_comp.Name in action_manager.mounted_tcs
    if mounted_tc and not to_mounted or not mounted_tc and to_mounted:
        return False
    assist_tc_action_cont = assistant_comp.findBehaviour('TaskContainer')  # type: vcActionContainer
    task_action.Message = prop_name
    task_action.AssistComponent = comp
    task_action.send(assist_tc_action_cont)
    return True


def assist_wait_for_ready(assisted_comp):
    # type: (vcComponent) -> None
    stats_manager.set_state('Blocked')
    ready_p = assisted_comp.getProperty('Assist::Ready')
    ready_p.OnChanged = activate_trigger
    condition(lambda: ready_p.Value)
    ready_p.OnChanged = None
    return


def assist_resolve_move_target(from_wpm, to_wpm):
    # type: (vcMatrix, vcMatrix) -> vcMatrix
    # substacting from and to pos vectors should form a vector pointing to the desired facing direction
    target_wpm = vcMatrix.new(from_wpm)

    # level to lowest matrix (project on floor level)
    if target_wpm.P.Z < to_wpm.P.Z:
        to_wpm = vcMatrix.new(to_wpm)
        to_wpm.translateAbs(0, 0, target_wpm.P.Z - to_wpm.P.Z)
    elif target_wpm.P.Z > to_wpm.P.Z:
        target_wpm.translateAbs(0, 0, to_wpm.P.Z - target_wpm.P.Z)

    from_to_vec = to_wpm.P - target_wpm.P
    hdg = math.atan2(from_to_vec.Y, from_to_vec.X) * 57.2957795  # calculate tangent angle (in degrees)
    l = from_to_vec.length()
    d = l - 10.0 if l < 1000.0 else 1000.0
    if l > 0:
        from_to_vec *= 1.0 - d / l
    else:
        from_to_vec = vcVector.new()

    target_wpm.translateAbs(from_to_vec.X, from_to_vec.Y, from_to_vec.Z)
    target_wpm.setWPR(0.0, 0.0, hdg)  # aling with from_to_vec
    return target_wpm


def is_cart(tool_comp):
    if not tool_comp:
        return False
    type_prop = tool_comp.getProperty('ToolType')
    if type_prop and type_prop.Value == 'Cart':
        return True
    return False


def get_cart_interface(find_last=True):
    # Find cart interface in the train
    iface = comp.findBehaviour('Child')
    while find_last and iface and iface.IsConnected:
        cart = iface.ConnectedComponent
        iface = cart.findBehaviour('Child')
    return iface


def get_first_cart():
    iface = get_cart_interface(False)
    if iface and iface.IsConnected:
        return iface.ConnectedComponent
    return None


def get_connected_carts(cart_comp):
    # Find all the carts in the train, input is any cart in the chain
    carts = []
    if not cart_comp or not is_cart(cart_comp):
        return carts
    cart = cart_comp
    while cart.Parent and is_cart(cart.Parent.Component):
        cart = cart.Parent.Component
    carts.append(cart)
    iface = cart.findBehaviour('Child')
    while iface and iface.IsConnected:
        cart = iface.ConnectedComponent
        carts.append(cart)
        iface = cart.findBehaviour('Child')
    return carts


def get_cart_pattern(cart_comp, prod_name):
    # type: (vcComponent, string) -> Pattern
    ref_node = find_node(cart_comp, 'TransportNode')
    if not ref_node:
        ref_node = cart_comp
    pattern_name = cart_comp.Name + '_' + prod_name
    type_drop_down_p = cart_comp.getProperty('Transport::ProductType')
    if not prod_name in type_drop_down_p.StepValues:
        pattern_name = cart_comp.Name + '_DEFAULT'
    pattern = pattern_manager.get_pattern(pattern_name)
    if pattern == pattern_manager.default_pattern:
        # Create pattern
        if pattern_name == cart_comp.Name + '_DEFAULT':
            ref_pos_frame = find_feature(cart_comp, 'DEFAULT_location')
            pattern_p = cart_comp.getProperty('Transport::Pattern')
            pattern_step_p = cart_comp.getProperty('Transport::PatternStep')
            keep_ori_p = cart_comp.getProperty('Transport::KeepOrientation')
        else:
            ref_pos_frame = find_feature(cart_comp, prod_name + '_location')
            pattern_p = cart_comp.getProperty('Transport::Pattern_' + prod_name)
            pattern_step_p = cart_comp.getProperty('Transport::PatternStep_' + prod_name)
            keep_ori_p = cart_comp.getProperty('Transport::KeepOrientation_' + prod_name)
        pattern = Pattern(pattern_name, ref_node, ref_pos_frame.NodePositionMatrix, pattern_p.Value,
                          pattern_step_p.Value, keep_ori_p.Value)
        pattern_manager.add_pattern(pattern)
    return pattern


def cart_has_capacity(cart_comp, prod_name):
    # Find cart interface in the train
    pattern = get_cart_pattern(cart_comp, prod_name)
    if pattern != pattern_manager.default_pattern:
        for slot in pattern.slots:
            if not slot.component:
                return True
    return False


def assing_to_cart(cart_comp, product_comp, tool_mode, tool_name):
    # Find the cart in train which mathces given tool_mode and tool_name and which has capacity
    assigned_cart = None
    carts = get_connected_carts(cart_comp)
    if tool_mode == 'Tool Name':
        carts = [x for x in carts if tool_name in x.Name]
    for cart in carts:
        if cart_has_capacity(cart, product_comp.Product.ProductType.Name):
            assigned_cart = cart
            break
    if not assigned_cart:
        assigned_cart = carts[0]
    return assigned_cart


def get_cart_offset(cart_comp):
    if not cart_comp:
        return None
    offset_prop = cart_comp.getProperty('OffsetResourceLocation')
    if not offset_prop or not offset_prop.Value:
        return None
    offset = vcVector.new()
    iface = comp.findBehaviour('Child')
    while iface and iface.IsConnected:
        cart = iface.ConnectedComponent
        if iface.Sections and iface.Sections[0].Frame:
            iface_pos = iface.Parent.PositionMatrix * iface.Sections[0].Frame.NodePositionMatrix
            offset = offset - iface_pos.P
        connected_section = iface.ConnectedToSections[0]
        if connected_section.Frame:
            iface_pos = connected_section.Frame.NodePositionMatrix
            offset = offset + iface_pos.P
        if cart == cart_comp:
            break
        iface = cart.findBehaviour('Child')
    return offset


def get_cart_for_product(product_comp):
    parent = product_comp.Parent
    while parent.Parent and parent.Parent.Component:
        parent_comp = parent.Component
        if is_cart(parent_comp):
            return parent_comp
        parent = parent_comp.Parent
    return None


#### Helper fuctions #####
def reserve_idle_location(accepted_locations=[]):
    global last_idle_chrg_loc
    # filter out non-accepted locations (defined by mission step filter)
    locations = idle_locations if not accepted_locations else accepted_locations

    # sort by nearest
    comp_wpm = travel_controller.interpolate(sim.SimTime)
    locations.sort(key=lambda c: (comp_wpm.P - c.WorldPositionMatrix.P).length())
    for loc in locations:
        if loc.Occupied < loc.Capacity:
            break
    else:
        # idle position not found!
        # will idle in place
        return None

    last_idle_chrg_loc = loc
    loc.Occupied += 1  # reserve location
    return loc


def reserve_charging_location(accepted_locations=[]):
    global last_idle_chrg_loc
    if not power_manager.enabled:
        return None

    # check first prioritized
    priority_list = []
    if power_manager.charge_priority_p.Value == 'Local Priority Order':
        priority_list = power_manager.local_priority_p.Value
    elif power_manager.charge_priority_p.Value == 'Global Priority Order':
        prop = action_manager.connected_controller.getProperty('ChargerPriorityOrder')
        if prop and prop.Value:
            priority_list = prop.Value

    loc = None
    for c in priority_list:
        if ((accepted_locations and c in accepted_locations) or (c in charging_stations)) and (c.Occupied < c.Capacity):
            loc = c
            break

    if not loc:
        # filter out non-accepted locations (defined by mission step filter)
        locations = charging_stations if not accepted_locations else accepted_locations

        # sort by nearest
        comp_wpm = travel_controller.interpolate(sim.SimTime)
        locations.sort(key=lambda c: (comp_wpm.P - c.WorldPositionMatrix.P).length())
        for loc in locations:
            if loc.Occupied < loc.Capacity:
                break
        else:
            # charging position not found!
            return None

    last_idle_chrg_loc = loc
    loc.Occupied += 1  # reserve location
    return loc


def release_idle_or_charging_location():
    global last_idle_chrg_loc
    if last_idle_chrg_loc:
        last_idle_chrg_loc.Occupied -= 1
        last_idle_chrg_loc = None
    debug('release_idle_or_charging_location', simtime=True)


def return_feature_world_location(node, feature_name):
    fea = node.getFeature(feature_name)
    if fea:
        node.update()
        return node.WorldPositionMatrix * fea.NodePositionMatrix
    return None


def is_world_root(node):
    return (node.Parent == None)


def find_feature(comp, feat_name):
    # Workaround for vcNode.findFeature() which can return feat from child component
    nodes = [comp]
    while nodes:
        node = nodes.pop(0)
        if node.Component.Name != comp.Name:
            continue
        feat = node.getFeature(feat_name)
        if feat:
            return feat
        nodes.extend(node.Children)
    return None


def find_node(comp, node_name):
    # Workaround for vcNode.findNode() which can return node from child component
    nodes = [comp]
    while nodes:
        node = nodes.pop(0)
        if node.Name == node_name:
            return node
        if node.Component.Name != comp.Name:
            continue
        nodes.extend(node.Children)
    return None


def activate_trigger(*args):
    evaluateCondition()


def validate_property(component, prop_type, prop_name, default_val=None, visible=True, step_values=[], unit='',
                      read_only=False):
    prop = component.getProperty(prop_name)
    if prop and prop.Type != prop_type:
        prop = component.deleteProperty(prop)
    if prop and step_values and prop.StepValues != step_values:
        prop.StepValues = step_values
        prop.Value = prop.StepValues[0]
    if not prop:
        if step_values:
            prop = component.createProperty(prop_type, prop_name, VC_PROPERTY_STEP)
            prop.StepValues = step_values
            prop.Value = prop.StepValues[0]
        else:
            prop = component.createProperty(prop_type, prop_name)
        if default_val is not None:
            prop.Value = default_val
        if unit:
            prop.Quantity = unit
        prop.IsVisible = visible
    if read_only:
        prop.WritableWhenConnected = False
        prop.WritableWhenDisconnected = False
    return prop


######################################################
##  ROBOT ARM CONTROLS
######################################################

robot_config_prop = comp.getProperty('Robot::Configuration')
robot_flange_tcp_prop = comp.getProperty('Robot::FlangeTCP')
robot_default_tcp_prop = comp.getProperty('Robot::DefaultTCP')


def update_arm_config_prop_list(robot):
    # If not vcRobot2 instance (in case of on_attach event)
    if type(robot) is not robohelp.vcRobot2:
        robot = robohelp.getRobot(robot)

    if robot_config_prop.Value not in robot.ConfigurationsList:
        robot_config_prop.StepValues = robot.ConfigurationsList
        robot_config_prop.Value = robot.ConfigurationsList[arm_default_config_index(robot)]


def arm_default_config_index(robot):
    if robot.Component.Name.startswith('UR'):
        return 1
    return 0


def toggle_robot_arm_tab(robot=None):
    val = True if robot else False
    robotProps = (p for p in comp.Properties if p.Name.startswith('Robot::'))
    for prop in robotProps:
        prop.IsVisible = val


def on_attach(type, node1, node2):
    if type == VC_NODE_ADD_FIRST_CHILD:
        if node1.findBehavioursByType(VC_ROBOTCONTROLLER):
            toggle_robot_arm_tab(node1)
            update_arm_config_prop_list(node1)


comp.OnNodeConfigurationChange = on_attach

##################################
##End of robot related functions##
##################################


######################################################
##  LOCATION AND PATTERNS
######################################################

location_ui_update_lock = False


def location_ui_clear_all(prop):
    # clear all frames from node. Frames define teached location for product type
    res = app.messageBox('All locations will be deleted. Are you sure?', 'Remove all locations',
                         VC_MESSAGE_TYPE_WARNING, VC_MESSAGE_BUTTONS_OKCANCEL)
    if res != VC_MESSAGE_RESULT_OK:
        return

    tf = find_feature(comp, 'Locations')
    if tf:
        for fea in tf.Children:
            if fea.Name.startswith('DEFAULT_'):
                continue
            fea.delete()

    delete_props = [p for p in comp.Properties if
                    not p.Name.endswith('_DEFAULT') and
                    (p.Name.startswith('Transport::KeepOrientation_') or
                     p.Name.startswith('Transport::PatternStep_') or
                     p.Name.startswith('Transport::Pattern_'))]

    for prop in delete_props:
        comp.deleteProperty(prop)

    location_ui_type_drop_down_p.StepValues = ['Default <Any>', 'Add New...']
    location_ui_type_drop_down_p.Value = location_ui_type_drop_down_p.StepValues[0]


def location_ui_delete(prop):
    prod_type_name = location_ui_type_drop_down_p.Value
    if prod_type_name in ('Default <Any>', 'Add New...'):
        return
    fea = find_feature(comp, prod_type_name + '_loc_offset')
    fea.delete()

    delete_props = [p for p in comp.Properties if
                    p.Name.startswith('Transport::') and p.Name.endswith('_' + prod_type_name)]
    for prop in delete_props:
        comp.deleteProperty(prop)
    type_vals = location_ui_type_drop_down_p.StepValues  # type: List[str]
    type_vals.pop(type_vals.index(prod_type_name))
    location_ui_type_drop_down_p.StepValues = type_vals
    location_ui_type_drop_down_p.Value = location_ui_type_drop_down_p.StepValues[0]


def location_ui_get_frame_pos(prod_type_name):
    frame_name = prod_type_name + '_location'
    frame = find_feature(comp, frame_name)
    if not frame:
        raise Exception('ERROR: Location frame "{}" not found!'.format(frame_name))
    return frame.NodePositionMatrix


def location_ui_create_frame(prod_type_name, loc_mtx):
    locs = find_feature(comp, 'Locations')
    offset_name = prod_type_name + '_loc_offset'
    frame_name = prod_type_name + '_location'

    offset = find_feature(comp, offset_name)
    frame = find_feature(comp, frame_name)
    if offset:
        offset.delete()
    if frame:
        frame.delete()

    offset = locs.createFeature(VC_TRANSFORM, offset_name)
    frame = offset.createFeature(VC_FRAME, frame_name)
    offset.Expression = 'Tx({}).Ty({}).Tz({}).Rx({}).Ry({}).Rz({})'.format(loc_mtx.P.X, loc_mtx.P.Y, loc_mtx.P.Z,
                                                                           loc_mtx.WPR.X, loc_mtx.WPR.Y, loc_mtx.WPR.Z)
    offset.rebuild()


def location_ui_unfocus_frames():
    locs = find_feature(comp, 'Locations')
    for loc in locs.Children:
        if loc.Expression == 'Transport::TransportLocation':
            loc_mtx = comp.getProperty('Transport::TransportLocation').Value
            loc.Expression = 'Tx({}).Ty({}).Tz({}).Rx({}).Ry({}).Rz({})'.format(loc_mtx.P.X, loc_mtx.P.Y, loc_mtx.P.Z,
                                                                                loc_mtx.WPR.X, loc_mtx.WPR.Y,
                                                                                loc_mtx.WPR.Z)


def location_ui_focus_on_frame(prod_type_name):
    loc_name = prod_type_name + '_loc_offset'
    loc_offset = find_feature(comp, loc_name)
    if loc_offset:
        loc_offset.Expression = 'Transport::TransportLocation'
        loc_offset.rebuild()
    else:
        raise Exception('ERROR: Location frame Transform "{}" not found!'.format(loc_name))


def location_ui_prod_picked(picked):
    if not picked.Value:
        return
    product = picked.Value.Component.Product
    if not product:
        print
        'ERROR: Selected component is not a product instance'
        picked.Value = None
        return
    comp.getProperty('Transport::ProductTypeName').Value = product.ProductType.Name


def location_ui_drop_down(prop):
    global location_ui_update_lock
    location_ui_update_lock = True  # lock updates
    location_ui_unfocus_frames()
    if prop.Value == 'Add New...':
        vis = False
        update_enabled = False
        comp.getProperty('Transport::ProductTypeName').Value = ''
        comp.getProperty('Transport::PickProduct').Value = None
    else:
        vis = True
        type_name = 'DEFAULT' if prop.Value == 'Default <Any>' else prop.Value
        update_enabled = type_name != 'DEFAULT'
        comp.getProperty('Transport::TransportLocation').Value = location_ui_get_frame_pos(type_name)
        # comp.getProperty('Transport::TransportLocation').Value = comp.getProperty('Transport::TransportLocation_' + type_name).Value
        location_ui_keep_ori_p.Value = comp.getProperty('Transport::KeepOrientation_' + type_name).Value
        location_ui_pattern_p.Value = comp.getProperty('Transport::Pattern_' + type_name).Value
        location_ui_pattern_step_p.Value = comp.getProperty('Transport::PatternStep_' + type_name).Value
        location_ui_focus_on_frame(type_name)

    comp.getProperty('Transport::PickProduct').IsVisible = not vis
    comp.getProperty('Transport::ProductTypeName').IsVisible = not vis
    comp.getProperty('Transport::Add Location').IsVisible = not vis
    comp.getProperty('Transport::TransportLocation').IsVisible = vis
    location_ui_update_loc_p.IsVisible = vis
    # location_ui_update_loc_p.WritableWhenConnected = update_enabled
    # location_ui_update_loc_p.WritableWhenDisconnected = update_enabled
    # location_ui_update_loc_p.WritableWhenSimulating = update_enabled
    location_ui_keep_ori_p.IsVisible = vis
    location_ui_pattern_step_p.IsVisible = vis
    location_ui_pattern_p.IsVisible = vis
    location_ui_del_loc_p.IsVisible = vis
    location_ui_del_loc_p.WritableWhenConnected = update_enabled
    location_ui_del_loc_p.WritableWhenDisconnected = update_enabled
    location_ui_del_loc_p.WritableWhenSimulating = update_enabled
    location_ui_clear_p.IsVisible = vis
    location_ui_update_lock = False  # unlock updates


def location_ui_add_new(prop):
    new_type = comp.getProperty('Transport::ProductTypeName').Value
    if not new_type or new_type == 'DEFAULT':
        return
    types = location_ui_type_drop_down_p.StepValues
    if new_type not in types:
        types.insert(-1, new_type)
        location_ui_type_drop_down_p.StepValues = types

        trans_node = comp.findNode('TransportNode')
        if not trans_node:
            trans_node = comp

        product_node = location_ui_pick_prod_p.Value
        if product_node:
            product_node.update()
            trans_node.update()
            loc_mtx = trans_node.InverseWorldPositionMatrix * product_node.Component.WorldPositionMatrix
        else:
            loc_mtx = vcMatrix.new()

        location_ui_create_frame(new_type, loc_mtx)

        # new_p = comp.createProperty(VC_MATRIX, 'Transport::TransportLocation_' + new_type)
        # new_p.IsVisible = False
        # new_p.Value = loc_mtx
        new_p = comp.createProperty(VC_BOOLEAN, 'Transport::KeepOrientation_' + new_type)
        new_p.IsVisible = False
        new_p = comp.createProperty(VC_VECTOR, 'Transport::Pattern_' + new_type)
        new_p.IsVisible = False
        new_p.Value = vcVector.new(1, 1, 1)
        new_p = comp.createProperty(VC_VECTOR, 'Transport::PatternStep_' + new_type)
        new_p.IsVisible = False
    location_ui_type_drop_down_p.Value = new_type


def location_ui_update_location(prop):
    prod_type_name = location_ui_type_drop_down_p.Value
    if prod_type_name == 'Default <Any>':
        prod_type_name = 'DEFAULT'

    trans_node = comp.findNode('TransportNode')
    if not trans_node:
        trans_node = comp

    if not config.cont.ComponentCount and prod_type_name == 'DEFAULT':
        print
        'ERROR: No products on-board'
        return

    for c in config.cont.Components:
        if not c.Product:
            continue
        if c.Product.ProductType.Name == prod_type_name or prod_type_name == 'DEFAULT':
            comp.update()
            c.update()
            comp.getProperty(
                'Transport::TransportLocation').Value = trans_node.InverseWorldPositionMatrix * c.WorldPositionMatrix
            break
    else:
        print
        'ERROR: No product instances of type "{}" found. Please make sure that the resource has the product on-board.'.format(
            prod_type_name)
        return

    if sim.SimTime > 0:
        print
        'Location updated. Changes become effective after simulation reset.'


def location_ui_update_values(prop):
    if location_ui_type_drop_down_p.Value == 'Add New...' or location_ui_update_lock:
        return
    type_name = 'DEFAULT' if location_ui_type_drop_down_p.Value == 'Default <Any>' else location_ui_type_drop_down_p.Value
    pattern_p = location_ui_pattern_p
    pat_val = pattern_p.Value
    pattern_p.Value = vcVector.new(
        1 if pat_val.X == 0 else pat_val.X,
        1 if pat_val.Y == 0 else pat_val.Y,
        1 if pat_val.Z == 0 else pat_val.Z)

    comp.getProperty('Transport::KeepOrientation_' + type_name).Value = location_ui_keep_ori_p.Value
    comp.getProperty('Transport::PatternStep_' + type_name).Value = location_ui_pattern_step_p.Value
    comp.getProperty('Transport::Pattern_' + type_name).Value = pattern_p.Value


location_ui_type_drop_down_p = comp.getProperty('Transport::ProductType')
location_ui_type_drop_down_p.OnChanged = location_ui_drop_down
location_ui_add_new_type_p = comp.getProperty('Transport::Add Location')
location_ui_add_new_type_p.OnChanged = location_ui_add_new
location_ui_pick_prod_p = comp.getProperty('Transport::PickProduct')
location_ui_pick_prod_p.OnChanged = location_ui_prod_picked
location_ui_keep_ori_p = comp.getProperty('Transport::KeepOrientation')
location_ui_keep_ori_p.OnChanged = location_ui_update_values
location_ui_pattern_step_p = comp.getProperty('Transport::PatternStep')
location_ui_pattern_step_p.OnChanged = location_ui_update_values
location_ui_pattern_p = comp.getProperty('Transport::Pattern')
location_ui_pattern_p.OnChanged = location_ui_update_values
location_ui_update_loc_p = comp.getProperty('Transport::Update From Product')
location_ui_update_loc_p.OnChanged = location_ui_update_location
location_ui_del_loc_p = comp.getProperty('Transport::Delete Location')
location_ui_del_loc_p.OnChanged = location_ui_delete
location_ui_clear_p = comp.getProperty('Transport::Delete All Locations')
location_ui_clear_p.OnChanged = location_ui_clear_all


######################################################
##  End of LOCATION AND PATTERNS
######################################################

class Config(object):
    def __init__(self):
        comp = getComponent()
        self.cont = validate_property(comp, 'Ref<Behaviour>', 'Config::DefaultContainer', visible=False).Value
        self.tool_cont = validate_property(comp, 'Ref<Behaviour>', 'Config::ToolContainer', visible=False).Value
        self.actuator_control_enabled = validate_property(comp, VC_BOOLEAN, 'Config::ActuatorControl', False,
                                                          visible=False).Value
        self.human_control_enabled = validate_property(comp, VC_BOOLEAN, 'Config::HumanControl', False,
                                                       visible=False).Value
        self.power_manager_enabled = validate_property(comp, VC_BOOLEAN, 'Config::PowerManager', False,
                                                       visible=False).Value

        if not self.cont or not self.tool_cont:
            raise Exception(comp.Name, 'Configuration failed. Please check properties under Config::')


class HumanAnimationControl(object):
    def __init__(self, travel_controller, executor_name=''):
        # type: (TravelController, str) -> None
        comp = getComponent()
        self._sim = getSimulation()
        self.travel_controller = travel_controller
        self.executor = HumanAnimationControl.get_executor(executor_name)

        self.routines = set()
        prefixes = set()
        if self.executor:
            for routine_name in (r.Name for r in self.executor.Program.Routines):
                self.routines.add(routine_name)
                prefixes.add('_'.join(routine_name.split('_')[:-1]))
        self.routine_prefixes = tuple(prefixes)

        self.active = comp.SimulationLevel != VC_SIMULATION_FAST and self.executor
        self.current_animation = ''
        self.lock = False  # set to True to prevent animation changes

    def on_sim_lvl_change(self, lvl):
        self.active = lvl != VC_SIMULATION_FAST and self.executor
        if not self.active and self._sim.IsRunning:
            self.executor.callRoutine('Default_Stand', False)

    def animate(self, anim='', default_anim='', lock=False):
        # type: (float, Optional[str], Optional[str], Optional[bool]) -> bool
        # Returns True if anim or default_anim was found and executed

        if not self.active:
            return

        if lock:
            self.lock = True

        if not anim and not self.lock:
            # called from OnSimUpdate. Prevent animation switching while picking/placing by setting self.lock
            if self.travel_controller.active_path:
                move = self.travel_controller.active_path.active_move if not self.travel_controller.active_path.paused else None
            elif self.travel_controller.active_move:
                move = self.travel_controller.active_move
            else:
                move = None

            if not move:
                pose = '_Stand'  # move ended
            elif move.turn_in_place and self._sim.SimTime < move.start_time + move.rotation_time:
                pose = '_Stand'  # turning in place
            else:
                pose = '_Walk'

            if current_tool:
                anim = self.get_tool_anim_prefix(current_tool.Name) + pose
                default_anim = 'Default' + pose + 'WithTool'
            elif config.cont.Components:
                anim = config.cont.Components[0].Product.ProductType.Name + pose
                default_anim = 'Default' + pose + 'WithPart'
            else:
                anim = 'Default' + pose

        if anim not in self.routines:
            anim = default_anim

        if anim and self.current_animation != anim:
            self.current_animation = anim
            self.executor.callRoutine(anim, False)
            return True
        return False

    def get_tool_anim_prefix(self, tool_name):
        # type: (str) -> str
        for i, prefix in enumerate(self.routine_prefixes):
            if prefix in tool_name:
                return self.routine_prefixes[i]
        return ''

    @staticmethod
    def get_executor(name=''):
        comp = getComponent()
        if name:
            executor = comp.findBehaviour(name)
        else:
            executors = comp.findBehavioursByType(VC_ROBOTEXECUTOR)
            executor = executors[0] if executors else None
        return executor

    @staticmethod
    def hide_joint_properties():
        executor = HumanAnimationControl.get_executor()
        if not executor:
            return
        for joint in executor.Controller.Joints:
            comp.getProperty(joint.Name).IsVisible = False


class ActuatorControl(object):
    CONF_AT_TARGET = 'Target'
    CONF_AT_APPROACH = 'Approach'
    CONF_AT_EXIT = 'Exit'
    CONF_NA = 'N/A'
    CONF_EXTEND_TO_MAX = 'MaxValue'
    CONF_EXTEND_TO_TARGET = 'ProductLevel'

    def __init__(self, node, servo_controller_name=''):
        # type: (vcServoController) -> None

        if servo_controller_name:
            self.servo = comp.findBehaviour(servo_controller_name)
        else:
            servos = comp.findBehavioursByType(VC_SERVOCONTROLLER)
            self.servo = servos[0] if servos else None

        if self.servo:
            if not self.servo.Joints:
                print
                comp.Name, '- ERROR: No joints defined in servo controller "{}" (ActuatorControl)'.format(
                    self.servo.Name)
                return

            self.node = node
            self._comp = self.servo.Component
            self.load_config()
            self.joint = self.servo.Joints[0]
            self.max_value = None
            self.buffer_container = comp.findBehaviour("BufferContainer")
            self.buffer_data = {}
            self.buffer_node = comp.findNode("Buffer")
            if self.buffer_node:
                buffer_node_vector = self.buffer_node.PositionMatrix.P
                for level in range(comp.Levels):
                    vector = vcVector.new(buffer_node_vector)
                    vector.Z += level * comp.LevelHeight
                    self.buffer_data[level] = [level, vector, None]

            for prop in (p for p in self.joint.Dof.Properties if p.Name == 'MaxLimit'):
                self.max_value = prop

            if self.conf_extend_to == ActuatorControl.CONF_EXTEND_TO_MAX and not self.max_value:
                print
                comp.Name, '- ERROR: Unable to read MaxValue for joint "{}" (ActuatorControl)'.format(self.joint.Name)

    def get_free_slot(self):
        for level in range(comp.Levels):
            slot_data = self.buffer_data[level]
            if slot_data[2] == None:
                return slot_data

    def get_contained_slot(self, part):
        for level in range(comp.Levels):
            slot_data = self.buffer_data[level]
            if slot_data[2] == part:
                return slot_data

    def load_config(self):
        _comp = self.servo.Component
        step_vals_app = [ActuatorControl.CONF_AT_APPROACH, ActuatorControl.CONF_AT_TARGET, ActuatorControl.CONF_NA]
        step_vals_exit = [ActuatorControl.CONF_AT_TARGET, ActuatorControl.CONF_AT_EXIT, ActuatorControl.CONF_NA]
        step_vals_extd = [ActuatorControl.CONF_EXTEND_TO_TARGET, ActuatorControl.CONF_EXTEND_TO_MAX]
        self.conf_load_extend = validate_property(_comp, VC_STRING, 'Actuator::LoadExtendAt', visible=False,
                                                  step_values=step_vals_app).Value
        self.conf_load_retract = validate_property(_comp, VC_STRING, 'Actuator::LoadRectractAt', visible=False,
                                                   step_values=step_vals_exit).Value
        self.conf_unload_extend = validate_property(_comp, VC_STRING, 'Actuator::UnloadExtendAt', visible=False,
                                                    step_values=step_vals_app).Value
        self.conf_unload_retract = validate_property(_comp, VC_STRING, 'Actuator::UnloadRectractAt', visible=False,
                                                     step_values=step_vals_exit).Value
        self.conf_extend_to = validate_property(_comp, VC_STRING, 'Actuator::ExtendTo', visible=False,
                                                step_values=step_vals_extd).Value
        self.conf_transport_pos = self.servo.Joints[0].InitialValue

    def extend(self, pos_mtx):
        # type: (vcMatrix) -> None
        if self.conf_extend_to == ActuatorControl.CONF_EXTEND_TO_MAX and self.max_value:
            self.servo.move(self.max_value.CalculatedValue)
        elif comp.getProperty("Levels"):
            comp_matrix = comp.InverseWorldPositionMatrix * pos_mtx
            node_matrix = self.node.InverseWorldPositionMatrix * pos_mtx
            x = comp_matrix.P.X
            y = comp_matrix.P.Y
            z = node_matrix.P.Z
            xy_length = math.sqrt(x * x + y * y)

            ''' It doesn't seem necessary to calculate the actual angle; only three angles are needed.
      angle = math.atan2(y, x) * 180.0 / math.pi      
      #print y, x, angle
      xy_length = math.sqrt(x * x + y * y)      
      if xy_length < 1.0 or abs(angle) < 1.0 or abs(180.0 - angle) < 1.0 or abs(180.0 + angle) < 1.0:
        angle = 0.0
      '''

            if abs(y) > abs(x) and y > 0.0:
                angle = -90.0
            elif abs(y) > abs(x) and y < 0.0:
                angle = 90.0
            else:
                angle = 0.0
                print
                "%s: Multi-layers AMR loads and unloads from the side. Please modify ResourceLocation so that the AMR docks from the side" % comp.Name

            self.servo.moveJoint(1, angle)
            self.servo.moveJoint(0, self.joint.CurrentValue + z)
            self.servo.moveJoint(2, xy_length * 0.5)
            self.servo.moveJoint(3, 90.0)
        else:
            h = pos_mtx.P.Z - self.node.WorldPositionMatrix.P.Z
            self.servo.move(self.joint.CurrentValue + h)

    def retract(self):
        # type: () -> None
        self.servo.move(self.conf_transport_pos)

    def place_to_buffer(self, part):
        slot_data = self.get_free_slot()
        level = slot_data[0]
        vector = slot_data[1]

        self.servo.moveJoint(2, 0.0)
        z = vector.Z - (comp.InverseWorldPositionMatrix * self.node.WorldPositionMatrix).P.Z

        self.servo.moveJoint(1, 0.0)
        self.servo.moveJoint(0, self.servo.Joints[0].CurrentValue + z)
        x = (self.node.InverseWorldPositionMatrix * self.buffer_node.WorldPositionMatrix).P.X
        self.servo.moveJoint(2, x * 0.5)
        self.buffer_container.grab(part)
        self.servo.moveJoint(3, 0.0)
        self.servo.moveJoint(2, 0.0)
        self.buffer_data[level] = [level, vector, part]

    def pick_from_buffer(self, part):
        slot_data = self.get_contained_slot(part)
        level = slot_data[0]
        vector = slot_data[1]

        self.servo.moveJoint(1, 0.0)
        z = vector.Z - (comp.InverseWorldPositionMatrix * self.node.WorldPositionMatrix).P.Z
        self.servo.moveJoint(0, self.servo.Joints[0].CurrentValue + z)

        x = (self.node.InverseWorldPositionMatrix * self.buffer_node.WorldPositionMatrix).P.X
        self.servo.moveJoint(2, x * 0.5)
        config.cont.grab(part)
        self.servo.moveJoint(3, 90.0)
        self.servo.moveJoint(2, 0.0)
        # self.servo.moveJoint(1, 0.0)
        self.buffer_data[level] = [level, vector, None]


class PatternManager(object):
    def __init__(self):
        self.patterns = []
        self.default_pattern = None

    def add_pattern(self, pattern):
        # type: (Pattern) -> None
        self.patterns.append(pattern)

    def get_pattern(self, name):
        # type: (str) -> Pattern
        for pat in self.patterns:
            if pat.name == name:
                return pat
        return self.default_pattern

    def find_slot(self, component):
        # type: (vcComponent) -> Slot
        for pat in self.patterns:  # type: Pattern
            slot = pat.get_slot(component)
            if slot:
                return slot
        return None

    def set_default(self, pattern):
        # type: (Pattern) -> None
        self.default_pattern = pattern


class Pattern(object):
    def __init__(self, name, ref_node, ref_mtx, pattern, steps, keep_orientation):
        # type: (str, vcNode, vcMatrix, vcVector, vcVector, bool) -> None
        self.name = name
        self.ref_node = ref_node
        self.ref_mtx = ref_mtx
        self.slots = []
        self.keep_orientation = keep_orientation

        x_slots = range(int(pattern.X))
        y_slots = range(int(pattern.Y))
        z_slots = range(int(pattern.Z))

        for z in z_slots:
            for y in y_slots:
                for x in x_slots:
                    mtx = vcMatrix.new(ref_mtx)
                    mtx.translateRel(x * steps.X, y * steps.Y, z * steps.Z)
                    self.create_slot(mtx)

    def add_slot(self, slot):
        # type: (vcSlot) -> None
        self.slots.append(slot)

    def create_slot(self, pos_mtx):
        # type: (vcMatrix) -> Slot
        slot = Slot(self, pos_mtx)
        self.slots.append(slot)
        return slot

    def get_free_slot(self):
        # type: () -> Slot
        for slot in self.slots:
            if not slot.component:
                return slot

        print
        '{} WARNING: No slots available in pattern "{}". Creating a new slot...'.format(comp.Name, self.name)
        new_slot = self.create_slot(self.ref_mtx)
        return new_slot

    def get_slot(self, component):
        # type: (vcComponent) -> Optional[Slot]
        for slot in self.slots:
            if slot.component == component:
                return slot
        return None

    def get_world_pos_matrix(self, slot):
        # type: (Slot) -> vcMatrix
        self.ref_node.update()
        return self.ref_node.WorldPositionMatrix * slot.pos


class Slot(object):
    def __init__(self, pattern, pos_mtx):
        # type: (Pattern, vcMatrix) -> None
        self.pattern = pattern
        self.pos = vcMatrix.new(pos_mtx)
        self.component = None  # type: vcComponent

    def __str__(self):
        return 'X:{}, Y:{}, Z:{}, component: {}'.format(self.pos.P.X, self.pos.P.Y, self.pos.P.Z, self.component)


class ObjectInterpolatorManager(object):
    sim = getSimulation()

    def __init__(self, capacity_prop=None, auto_grab_to_target=False):
        # type: (vcProperty) -> None
        self.interpolators = []
        self.capacity_p = capacity_prop
        self._capacity_limit = bool(self.capacity_p)
        self.queue = deque()
        self.auto_grab = auto_grab_to_target

    def interpolate(self, sim_time=-1):
        if sim_time < 0:
            sim_time = ObjectInterpolatorManager.sim.SimTime
        for itp in self.interpolators:
            itp.interpolate(sim_time)
        if self.interpolators:
            # clean old interpolators
            if self.auto_grab:
                for oi in [i for i in self.interpolators if i.end_time <= sim_time]:
                    oi.to_container.grab(oi.component)
                    self.interpolators.remove(oi)
            else:
                self.interpolators = [i for i in self.interpolators if sim_time < i.end_time]
        if self._capacity_limit:
            self._update_queue()

    def _update_queue(self):
        while self.queue and len(self.interpolators) < self.capacity_p.Value:
            oi = self.queue.popleft()
            self.add(oi, from_queue=True)

    def add(self, interpolator, from_queue=False):
        # type: (ObjectInterpolator) -> None
        if self._capacity_limit:
            if self.capacity_p.Value <= len(self.interpolators):
                self.queue.append(interpolator)
                return
            if self.queue and not from_queue:
                self.queue.append(interpolator)
                interpolator = self.queue.popleft()

        if not interpolator.started:
            t = ObjectInterpolatorManager.sim.SimTime
            interpolator.start(t)
        self.interpolators.append(interpolator)

    def remove(self, interpolator):
        # type: (ObjectInterpolator) -> None
        self.interpolators.remove(interpolator)

    def wait_to_complete(self, interpolator=None):
        # type: (Optional[ObjectInterpolator]) -> None
        if interpolator:
            delay(interpolator.end_time - ObjectInterpolatorManager.sim.SimTime)
        else:
            delay(self.last_time_to_target(ObjectInterpolatorManager.sim.SimTime))
        self.interpolate(ObjectInterpolatorManager.sim.SimTime)

    def next_time_to_target(self, sim_time):
        if self.interpolators:
            return min(i.end_time for i in self.interpolators) - sim_time
        return -1.0

    def last_time_to_target(self, sim_time):
        if self.interpolators:
            return max(i.end_time for i in self.interpolators) - sim_time
        return -1.0


class ObjectInterpolator(object):
    __slots__ = (
    'component', 'to_container', 'from_mtx', 'to_mtx', 'transform', 'duration', 'velocity', 'r', 'rdot', 'start_time',
    'end_time', 'grab_to_world', 'started', 'from_path')
    WORLD_CONT = [b for b in sim.World.Behaviours if b.Type == VC_COMPONENTCONTAINER][0]
    PATH_TYPES = (VC_ONEWAYPATH, VC_TWOWAYPATH)

    def __init__(self, component, to_wpm, to_container, duration=-1.0, velocity=-1.0, grab_to_world=True):
        # type: (vcComponent, vcMatrix, vcContainer, Optional[float], Optional[float], Optional[bool]) -> None
        component.update()
        self.component = component
        self.to_container = to_container
        self.grab_to_world = grab_to_world
        self.from_path = not grab_to_world and self.component.Container.Type in ObjectInterpolator.PATH_TYPES
        if grab_to_world:
            self.from_mtx = component.WorldPositionMatrix
            self.to_mtx = to_wpm
        else:
            self.from_mtx = component.MovementOrigin if self.from_path else component.PositionMatrix
            self.to_mtx = component.InverseWorldPositionMatrix * to_wpm if self.from_path else component.Parent.InverseWorldPositionMatrix * to_wpm

        self.transform = component.InverseWorldPositionMatrix * to_wpm
        self.duration = duration
        self.velocity = velocity

        d = self.transform.P.length()
        if self.duration >= 0:
            self.velocity = d / duration
        elif self.velocity >= 0:
            self.duration = d / self.velocity
        else:
            raise Exception(comp.Name, 'ERROR - duration or velocity must be given')

        self.r = self.transform.getAxisAngle()
        self.rdot = self.r.W / self.duration
        self.started = False

    def start(self, sim_time):
        if self.grab_to_world:
            ObjectInterpolator.WORLD_CONT.grab(self.component)
        self.start_time = sim_time
        self.end_time = self.start_time + self.duration
        self.started = True

    def interpolate(self, sim_time):
        if not self.started:
            return

        if self.end_time <= sim_time:
            pos = self.to_mtx
        else:
            t = sim_time - self.start_time
            tf = vcMatrix.new()
            tf.rotateAbsV(self.r, self.rdot * t)
            tf.P = self.transform.P * (t / self.duration)
            pos = self.from_mtx * tf

        if self.from_path:
            self.component.MovementOrigin = pos
        else:
            self.component.PositionMatrix = pos


class StatisticsManager(object):
    def __init__(self, state_node=None):
        self.statistics = getComponent().findBehavioursByType(VC_STATISTICS)[0]
        self.state_node = state_node
        self.state_materials = {}
        if state_node:
            state_node.MaterialInheritance = VC_MATERIAL_FORCE_INHERIT
        self.power_manager = None  # set to update power status when state changes
        self.current_state_p = getComponent().getProperty('CurrentState')

    def define_states(self, states_and_mats):
        # type: (List[Tuple[str, enum_const, str]])
        states = []
        for name, state, mat_name in states_and_mats:
            states.append((name, state))
            self.state_materials[name] = app.findMaterial(mat_name) if mat_name else None

        # update/rewrite states only if definitions have changed
        rewrite = False
        for state in states:
            if state not in self.statistics.States:
                rewrite = True
                break
        if rewrite or len(self.statistics.States) != len(states):
            self.statistics.States = states

    def set_state(self, name):
        if self.power_manager:
            self.power_manager.update(True)
        if self.statistics.State == name:
            return
        self.statistics.State = name
        if self.state_node:
            mat = self.state_materials[name]
            if mat:
                self.state_node.NodeMaterial = mat
        if self.current_state_p:
            self.current_state_p.Value = name


class PowerManager(object):
    def __init__(self, statistics, enabled):
        # type: (vcStatistics) -> None
        global _PowerManager_enabled_p

        if not enabled:
            self.enabled = False
            return

        comp = getComponent()
        enabled_p = comp.getProperty('Power::Enabled')
        if enabled_p:
            _PowerManager_enabled_p = enabled_p
            _PowerManager_enabled_p.OnChanged = self._on_enable
        self.enabled = False if not enabled_p else enabled_p.Value
        self.stats = statistics
        self.PCap = comp.getProperty('Power::Capacity')  # PCap
        self.PInitCap = comp.getProperty('Power::InitialCapacity')  # PInitCap
        self.PBusy = comp.getProperty('Power::BusyConsumption_h')  # PBusy
        self.PPick = comp.getProperty('Power::PickingConsumption_h')  # PPick
        self.PPlace = comp.getProperty('Power::PlacingConsumption_h')  # PPlace
        self.PIdle = comp.getProperty('Power::IdleConsumption_h')  # PIdle
        self.PCharge = comp.getProperty('Power::ReChargeRate_h')  # PCharge
        self.PCurrent = comp.getProperty('Power::CurrentCapacity')  # PCurrent
        self.PCLimit = comp.getProperty('Power::ChargeUntilLimit')  # PCLimit
        self.PToLimit = comp.getProperty('Power::ToChargeLimit')  # PToLimit
        self.charge_priority_p = comp.getProperty('Power::ChargerPriority')
        self.local_priority_p = comp.getProperty('Power::ChargerPriorityOrder')
        self.charge_on_idle_p = comp.getProperty('Power::ChargeOnIdle')
        self.low_battery_p = comp.getProperty('Power::LowBattery')
        self.on_low_battery_p = comp.getProperty('Power::OnLowBattery')

        if self.enabled:
            self.PCurrent.Value = self.PInitCap.Value
            self.last_state_pow_lvl = self.PCurrent.Value
            self.last_state_time = 0.0

    def get_time_to_charging_limit(self, charge_until_limit=-1):
        if charge_until_limit == -1:
            charge_until_limit = self.PCLimit.Value
        return (((self.PCap.Value * charge_until_limit) - self.PCurrent.Value) / self.PCharge.Value) * 3600.0

    def get_time_to_fullcharge(self):
        return round((((self.PCap.Value) - self.PCurrent.Value) / self.PCharge.Value) * 3600.0, 2)

    def update(self, state_change=False):
        if not self.enabled:
            return

        # calculate delta for power
        if self.stats.State == 'Idle':
            d_pow = -self.PIdle.Value * ((sim.SimTime - self.last_state_time) / 3600.0)
        elif self.stats.State == 'Picking':
            d_pow = -self.PPick.Value * ((sim.SimTime - self.last_state_time) / 3600.0)
        elif self.stats.State == 'Placing':
            d_pow = -self.PPlace.Value * ((sim.SimTime - self.last_state_time) / 3600.0)
        elif self.stats.State == 'Charging':
            d_pow = self.PCharge.Value * ((sim.SimTime - self.last_state_time) / 3600.0)
        elif self.stats.State == 'Break':
            d_pow = 0
        else:  # Busy
            d_pow = -self.PBusy.Value * ((sim.SimTime - self.last_state_time) / 3600.0)

        self.PCurrent.Value = self.last_state_pow_lvl + d_pow
        if self.PCurrent.Value > self.PCap.Value:
            self.PCurrent.Value = self.PCap.Value  # limit to max

        if state_change:
            self.last_state_time = sim.SimTime
            self.last_state_pow_lvl = self.PCurrent.Value

        # charging required

        if self.PCurrent.Value < (self.PToLimit.Value * self.PCap.Value):
            if not self.low_battery_p.Value:
                self.low_battery_p.Value = True
            if action_manager.active_action != execute_charging:
                if action_manager.is_reserved():
                    return  # do not schedule charging during reservation
                action_manager.set_availability(False)  # block new tasks
                if execute_charging not in action_manager.custom_task_actions:
                    # schedule charging if not already scheduled
                    action_manager.schedule_custom_action(execute_charging)
                    if self.stats.State != 'Blocked':
                        evaluateCondition()  # excite onrun if e.g. at idle
        elif self.low_battery_p.Value:
            self.low_battery_p.Value = False

    def _on_enable(self, enabled_prop):
        self.enabled = enabled_prop.Value
        if self.enabled:
            self.last_state_time = sim.SimTime
            self.last_state_pow_lvl = self.PCurrent.Value


class ActionManager(object):
    UNASSISTED = 0
    LOAD_ASSISTED = 1
    UNLOAD_ASSISTED = 2
    LOAD_ASSISTANT = 3
    UNLOAD_ASSISTANT = 4

    def __init__(self, resource_interface=None, local_mode=False):
        self.available = True  # set this to False to block new tasks
        self.active_action = None
        self.interface = resource_interface
        self._comp = getComponent()

        self._can_recv_tasks_p = validate_property(self._comp, VC_BOOLEAN, 'CanReceiveTasks', True, False)
        self.tasks_completed_p = validate_property(self._comp, VC_BOOLEAN, 'TasksCompleted', False, False)
        self.available_capacity_p = validate_property(self._comp, VC_INTEGER, 'AvailableCapacity', 10, False)
        self.transport_capacity_p = validate_property(self._comp, VC_INTEGER, 'Transport::Capacity', 10, True)

        # Allow dynamically changing the maximum capacity of the resource
        # Item 32206
        self.oldCapacity = self.transport_capacity_p.Value

        def updateCapacity(_):
            self.available_capacity_p.Value += self.transport_capacity_p.Value - self.oldCapacity
            self.oldCapacity = self.transport_capacity_p.Value

        self.transport_capacity_p.OnChanged = updateCapacity

        self.reserved_to_p = validate_property(self._comp, 'List<Ref<Behaviour>>', 'ReservedToNodes', [], False)
        # self.prioritized_to_p = validate_property(self._comp, 'Ref<Behaviour>', 'PrioritizedNode', None, False)

        self.locked_source_node_p = validate_property(self._comp, 'Ref<Behaviour>', 'LockedSourceNode', visible=False)
        self.locked_destination_node_p = validate_property(self._comp, 'Ref<Behaviour>', 'LockedDestinationNode',
                                                           visible=False)

        self.assist_ready_p = validate_property(self._comp, VC_BOOLEAN, 'Assist::Ready', False, False)
        self.assist_load_target_p = validate_property(self._comp, VC_MATRIX, 'Assist::LoadTarget', vcMatrix.new(),
                                                      False)
        self.assist_load_cont_p = validate_property(self._comp, 'Ref<Behaviour>', 'Assist::LoadContainer', None, False)

        self.current_mission_id_p = validate_property(self._comp, VC_STRING, 'Mission::Id', '', False, read_only=False)
        self.occurrence_index_p = validate_property(self._comp, VC_INTEGER, 'Mission::Occurence', -1, False,
                                                    read_only=False)
        self.current_mission_step_index_p = validate_property(self._comp, VC_INTEGER, 'Mission::StepIndex', -1, False,
                                                              read_only=False)
        self.current_mission_step_type_p = validate_property(self._comp, VC_STRING, 'Mission::StepType', '', False,
                                                             read_only=False)
        self.reserved_tasks_id = validate_property(self._comp, VC_STRING, 'Mission::ReservedTasks', '', False,
                                                   read_only=False)
        self.mission_completed_p = validate_property(self._comp, VC_BOOLEAN, 'Mission::IsCompleted', True, False,
                                                     read_only=False)

        self.local_mode = local_mode
        self.initialize(connect=False)

    def initialize(self, connect=True, lifo=True):
        self.mission_step_actions = []
        self.current_mission_step_type_p.Value = ''
        self.current_mission_step_index_p.Value = -1
        self.work_task_actions = []
        self.move_actions = []
        self.collect_task_actions = []
        self.deliver_task_actions = []
        self.custom_task_actions = []  # holds function references
        self.task_count = 0  # custom actions are excluded from the count
        self.available = True
        self.lifo = lifo
        if comp.getProperty("Levels"):
            self.transport_capacity_p.Value = comp.Levels
        self.available_capacity_p.Value = self.transport_capacity_p.Value

        self._can_recv_tasks_p.Value = True
        self.tasks_completed_p.Value = False
        self.mission_completed_p.Value = False
        self.reserved_to_p.Value = []
        self.reserved_tasks = {}

        self._tc_trigger_signal = None
        self._tc_beh = None
        self.connected_controller = None

        self.locked_source_node_p.Value = None
        self.locked_destination_node_p.Value = None

        if not connect:
            return
        if self.local_mode:
            tc_comp = getComponent()
        else:
            if not self.interface:
                print
                self._comp.Name, 'WARNING: No interface in ActionManager.'
                return
            tc_comp = self.interface.ConnectedComponent
            if not tc_comp:
                print
                self._comp.Name, 'WARNING: Not connected to transport controller.'
                return

        self._tc_trigger_signal = tc_comp.findBehaviour('Trigger')
        if not self._tc_trigger_signal:
            print
            self._comp.Name, 'WARNING: "Trigger" signal not found from connected Transport Controller.'

        self._tc_beh = tc_comp.findBehavioursByType(VC_PYTHONTRANSPORTCONTROLLER)[0]
        self.connected_controller = tc_comp

        self.assist_ready_p.Value = False
        self.assistance = ActionManager.UNASSISTED

        self.mounted_tcs = set()
        for child in self._comp.ChildComponents:
            if child.findBehavioursByType(VC_PYTHONTRANSPORTCONTROLLER):
                self.mounted_tcs.add(child.Name)

    def set_availability(self, available):
        if self.available == available:
            return
        if DEBUG_MODE: print
        comp.Name, 'setting availability to ', available
        self.available = available
        self._can_recv_tasks_p.Value = available

    def actions_available(self):
        if mission_mode_on:
            # print comp.Name, 'actions_available:', len(self.mission_step_actions) > 0
            return len(self.mission_step_actions) > 0
        else:
            # print comp.Name, 'actions_available:', self.task_count > 0 or self.custom_task_actions
            return self.task_count > 0 or self.custom_task_actions

    def is_reserved(self):
        return bool(self.reserved_to_p.Value)

    def update_tasks(self):
        self._tc_trigger_signal.signal(True)

    def is_last_action(self):
        return self.task_count == 1

    def mission_is_completed(self):
        return len(self.mission_step_actions) == 0

    def schedule_custom_action(self, action_function):
        # custom action will be executed after all other available actions
        # action_function is a reference to callback function that is invoked when executed with manager as the argument
        self.custom_task_actions.append(action_function)

    def cancel_custom_action(self, action_function):
        if action_function in self.custom_task_actions:
            self.custom_task_actions.remove(action_function)

    def work_completed(self, work_action):
        # print 'WORK Complete!'
        self.pre_process_trigger(add_capacity=0)
        self._tc_beh.workDone(work_action.WorkId)
        self.post_process_trigger()

    def custom_task_completed(self, last_action):
        # signal to TC that the last delivery or a work task has completed
        self.tasks_completed_p.Value = True
        self._can_recv_tasks_p.Value = True if self.available else False
        self._tc_trigger_signal.signal(True)
        self.active_action = None
        # print 'Tasks completed!!!'

    def pre_process_trigger(self, assisting_comp=None, add_capacity=1):
        # type: (vcComponent, int) -> None
        # print 'PREPLACE'
        if self.is_last_action():
            # print 'IS LAST ACTION'
            self.tasks_completed_p.Value = True
            self._can_recv_tasks_p.Value = True if self.available else False

        self.available_capacity_p.Value += add_capacity
        if assisting_comp:
            # free assisted resource's capacity
            assisting_comp.getProperty('AvailableCapacity').Value += add_capacity

    def post_process_trigger(self):
        # print 'Waiting new tasks...'
        delay(0.001)  # wait for possible tasks after placing (triggered by container)
        # print 'action count:',self.task_count
        if self.is_last_action():  # no new actions, notify TC
            # print 'Exciting TC!!!'
            self.active_action = None
            self._tc_trigger_signal.signal(True)
            delay(0.001)  # wait for new tasks

    def get_next_transport_action(self):
        # type: () -> Tuple(str, vcAction)
        # Use to check what's next
        if self.collect_task_actions:
            return 'Collect', self.collect_task_actions[0]
        elif self.deliver_task_actions:
            return 'Deliver', self.deliver_task_actions[0]
        return '', None

    def get_next_action(self):
        # type: () -> Tuple(str, vcAction)
        # Use to check what's next
        if self.mission_step_actions:
            return 'MissionStep', self.mission_step_actions[0]
        elif self.work_task_actions:
            return 'Work', self.work_task_actions[0]
        elif self.collect_task_actions:
            return 'Collect', self.collect_task_actions[0]
        elif self.deliver_task_actions:
            return 'Deliver', self.deliver_task_actions[0]
        elif self.custom_task_actions:
            return 'Custom', self.custom_task_actions[0]
        return '', None

    def turn_direction(self):
        self.pos_mtx.rotateRelZ(180.0)
        print("direction_changed")

    def execute_next_action(self):
        if mission_mode_on:
            self.update_tasks()
            if not self.mission_is_completed():
                # self.assistance = ActionManager.UNASSISTED
                execute_pending_move()
                mission_step = self.mission_step_actions.pop(0)
                self.current_mission_step_index_p.Value += 1
                self.current_mission_step_type_p.Value = mission_step.Name
                debug('Executing mission step #{}: "{}"'.format(mission_step.action_index, mission_step.Name),
                      simtime=True)
                if mission_step.Name == 'Move':
                    self.active_action = self.execute_move_step
                    self.execute_move_step(mission_step)
                elif mission_step.Name == 'Collect':
                    self.active_action = self.execute_collect_step
                    self.execute_collect_step(mission_step)
                elif mission_step.Name == 'CollectTool':
                    self.active_action = self.execute_collect_tool_step
                    self.execute_collect_tool_step(mission_step)
                elif mission_step.Name == 'Deliver':
                    self.active_action = self.execute_deliver_step
                    self.execute_deliver_step(mission_step)
                elif mission_step.Name == 'DeliverTool':
                    self.active_action = self.execute_deliver_tool_step
                    self.execute_deliver_tool_step(mission_step)
                elif mission_step.Name == 'Wait':
                    self.active_action = self.execute_wait_step
                    self.execute_wait_step(mission_step)
                elif mission_step.Name == 'Work':
                    self.active_action = self.execute_work_step
                    self.execute_work_step(mission_step)
                elif mission_step.Name == 'Charge':
                    self.active_action = self.execute_charge_step
                    self.execute_charge_step(mission_step)
                elif mission_step.Name == 'WaitSignal':
                    self.active_action = self.execute_wait_signal_step
                    self.execute_wait_signal_step(mission_step)
                elif mission_step.Name == 'SendSignal':
                    self.active_action = self.execute_send_signal_step
                    self.execute_send_signal_step(mission_step)
                elif mission_step.Name == 'MoveJoint':
                    self.active_action = self.execute_move_joint_step
                    self.execute_move_joint_step(mission_step)
                elif mission_step.Name == 'RunRobotRoutine':
                    self.active_action = self.execute_run_robot_routine_step
                    self.execute_run_robot_routine_step(mission_step)
                else:
                    raise Exception('{}: unknown mission step!'.format(comp.Name))
                if self.mission_is_completed():
                    self.mission_completed()
            else:
                raise Exception('{}: no mission more steps!'.format(comp.Name))
        else:
            execute_pending_move()
            # set can receive tasks state
            if not self.collect_task_actions and self.deliver_task_actions or not self.available:
                self._can_recv_tasks_p.Value = False  # currently delivering or unavailable

            if self.is_last_action():
                debug('UNLOCKING...', simtime=True)
                self.locked_source_node_p.Value = None
                self.locked_destination_node_p.Value = None
            if self.work_task_actions:
                self.assistance = ActionManager.UNASSISTED
                self.active_action = self.execute_work_action
                self.execute_work_action(self.work_task_actions.pop(0))
            elif self.collect_task_actions:
                self.active_action = self.execute_pick_action
                task_action = self.collect_task_actions.pop(0)
                self._evaluate_assistance(task_action)
                self.execute_pick_action(task_action)
            elif self.deliver_task_actions:
                self.active_action = self.execute_place_action
                task_action = self.deliver_task_actions.pop(0)
                self._evaluate_assistance(task_action)
                self.execute_place_action(task_action)
            elif self.custom_task_actions:
                self.assistance = ActionManager.UNASSISTED
                self.active_action = self.custom_task_actions.pop(0)
                self.active_action(self)
                return  # do not count custom actions
            else:
                self.active_action = None
                error('ERROR - Out of task actions!', simtime=True)
                return 0
            self.task_count -= 1
            stats_manager.set_state('Idle')

    def _evaluate_assistance(self, task_action):
        if not task_action.TransportLink.LoadAssist and not task_action.TransportLink.UnloadAssist:
            self.assistance = ActionManager.UNASSISTED
        elif not task_action.AssistComponent or task_action.AssistComponent == self._comp:  # Assisted
            if task_action.TransportLink.LoadAssist and self.active_action in [self.execute_pick_action,
                                                                               self.execute_collect_step]:  # Added (Mission assistance)
                self.assistance = ActionManager.LOAD_ASSISTED
            elif task_action.TransportLink.UnloadAssist and self.active_action in [self.execute_place_action,
                                                                                   self.execute_deliver_step]:  # Added (Mission assistance)
                self.assistance = ActionManager.UNLOAD_ASSISTED
            else:
                self.assistance = ActionManager.UNASSISTED
        elif self.active_action == self.execute_pick_action:  # Assistant
            if task_action.Component.Parent.Component == task_action.AssistComponent:
                if task_action.TransportLink.UnloadAssist:
                    self.assistance = ActionManager.UNLOAD_ASSISTANT
            elif task_action.TransportLink.LoadAssist:
                self.assistance = ActionManager.LOAD_ASSISTANT

    def execute_idle_action(self):
        self.active_action = self.execute_idle
        self.execute_idle()

    def execute_pick_action(self, task_action):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_pick_action"-method not implemented!')

    def execute_place_action(self, task_action):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_place_action"-method not implemented!')

    def execute_work_action(self, task_action):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_work_action"-method not implemented!')

    def execute_idle(self):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_idle"-method not implemented!')

    def execute_charge_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_charge_step"-method not implemented!')

    def execute_collect_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_collect_step"-method not implemented!')

    def execute_collect_tool_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_collect_tool_step"-method not implemented!')

    def execute_deliver_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_deliver_step"-method not implemented!')

    def execute_deliver_tool_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_deliver_tool_step"-method not implemented!')

    def execute_move_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_move_step"-method not implemented!')

    def execute_move_joint_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_move_joint_step"-method not implemented!')

    def execute_run_robot_routine_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_run_robot_routine_step"-method not implemented!')

    def execute_wait_signal_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_wait_signal_step"-method not implemented!')

    def execute_send_signal_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_send_signal_step"-method not implemented!')

    def execute_wait_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_wait"-method not implemented!')

    def execute_work_step(self, mission_step):
        raise NotImplementedError(self._comp.Name, 'ERROR - "execute_work_step"-method not implemented!')

    def get_reserved_tasks(self, action_index):
        if action_index in self.reserved_tasks:
            task_actions = self.reserved_tasks[action_index]
            del self.reserved_tasks[action_index]
            self.reserved_tasks_id.Value = str(self.reserved_tasks.keys())
            return task_actions
        else:
            return None

    def mission_completed(self):
        self.mission_completed_p.Value = True
        self._tc_trigger_signal.signal(True)
        self.active_action = None
        self.reserved_tasks = {}
        self.current_mission_step_index_p.Value = -1
        self.current_mission_step_type_p.Value = ''
        debug('Mission completed!', sim_time=True)

    def read_task_reference(self, header_action):
        info('Reading mission read_task_reference name "{}"" and message "{}"'.format(header_action.Name,
                                                                                      header_action.Message))

    def read_mission(self, header_action):
        if DEBUG_MODE: print
        '{}: reading mission with message "{}"'.format(comp.Name, header_action.Message)
        indices = header_action.Message[1:-1].split(',')  # excluding opening and closing brackets
        if not indices or indices == ['']:
            warning('No action indices defined in the message.', simtime=True)
            return
        for action in [header_action.Sender.getAction(int(i)) for i in indices]:
            self.mission_step_actions.append(action)
            # self.mission_step_count += 1

        header_action.Sender.deletePendingAction(header_action)
        self.mission_completed_p.Value = False
        self.current_mission_step_index_p.Value = 0
        # self.current_mission_step_type_p.Value = ''

    def cancel_mission(self):
        self.mission_step_actions = []
        self.mission_completed_p.Value = True
        self.current_mission_step_index_p.Value = 0
        self._tc_trigger_signal.signal(True)
        self.active_action = None
        self.reserved_tasks = {}
        self.current_mission_step_index_p.Value = -1
        self.current_mission_step_type_p.Value = ''
        if DEBUG_MODE: print
        comp.Name, 'Mission cancelled.'

    def read_tasks(self, header_action):
        ### In mission mode, the task are read and added in the tasks list, but not proceeded. Only when the mission is run, the tasks are executed.
        task_indices = header_action.Message[1:-1].split(',')  # excluding opening and closing brackets
        for task in [header_action.Sender.getAction(int(i)) for i in task_indices]:

            if DEBUG_MODE: print
            '{}: a new "{}"-task received.'.format(comp.Name, task.Name)
            assigned_indices_prop = task.getProperty('AssignedActionIndices')
            if assigned_indices_prop:
                # debug('[{}]: Reserving task #{}: "{}" => "{}", for Mission step indices #{}'.format(comp.Name,task.Index, task.SourceNode.Component.Name, task.TransportLink.Destination.Component.Name, reserved_indices_prop.Value))
                mission_step_indices = map(int, assigned_indices_prop.Value[1:-1].split(
                    ','))  # excluding opening and closing brackets
                for mission_step_index in mission_step_indices:
                    if mission_step_index in self.reserved_tasks:
                        self.reserved_tasks[mission_step_index] += [task]
                    else:
                        self.reserved_tasks[mission_step_index] = [task]
                self.reserved_tasks_id.Value = str(self.reserved_tasks.keys())
                # debug('{}: a new "{}"-task received with reserved action reserved_action_indices {}.'.format(comp.Name, task.Name, mission_step_indices))

            elif task.Name == 'Work':
                self.work_task_actions.append(task)
                self.task_count += 1
            else:
                # Transport
                self.collect_task_actions.append(task)
                if self.lifo:
                    self.deliver_task_actions.insert(0, task)
                else:
                    self.deliver_task_actions.append(task)
                self.task_count += 2

                if DEBUG_MODE: print
                comp.Name, 'read_tasks(): self.collect_task_actions', self.collect_task_actions, ' self.deliver_task_actions', self.deliver_task_actions, 'self.task_count', self.task_count

        header_action.Sender.deletePendingAction(header_action)


class PathSchema():
    ID = 0  # int
    REF = 1  # str
    CONTROLLER = 2  # str
    LINKS = 3  # str


class PathLinkSchema():
    ID = 0  # int
    FROM = 1  # vcVector
    TO = 2  # vcVector
    AREA = 3  # str
    TRAVEL_STATS = 4  # int
    AVOIDANCE_STATS = 5  # int


class NavArea(object):
    def __init__(self, component):
        # type: (vcComponent) -> None
        self.component = component
        self.limited_speed_p = component.getProperty('LimitSpeed')
        self.speed_limit_p = component.getProperty('SpeedLimit')
        self.limited_capacity_p = component.getProperty('LimitCapacity')
        self.capacity_p = component.getProperty('Capacity')
        self.used_capacity_p = component.getProperty('UsedCapacity')
        self.queue_p = component.getProperty('Queue')
        self.group = None

    def assign_group(self, group):
        # type: (NavAreaCapacityGroup) -> None
        self.group = group
        if group:
            group.add_area(self)

    def has_capacity(self):
        # type: () -> bool

        return not self.limited_capacity_p.Value or self.used_capacity_p.Value < self.capacity_p.Value

    def place_on_queue(self, resource):
        # type: (vcComponent) -> None
        self.queue_p.Value += [resource]

    def remove_from_queue(self):
        # type: (vcComponent) -> None
        self.queue_p.Value = self.queue_p.Value[1:]

    def in_queue(self, component):
        # type: (vcComponent) -> bool
        if component:
            return component in self.queue_p.Value
        return False

    def next_in_queue(self, resource):
        # type: (vcComponent) -> bool
        return self.queue_p.Value[0] == resource

    def index_in_queue(self, resource):
        # type: (vcComponent) -> bool
        return self.queue_p.Value.index(resource)


class NavAreaCapacityGroup(object):
    def __init__(self, component):
        self.component = component
        self.name = component.Name
        check_mode_options = ['Areas on the incoming resource route must have capacity',
                              'All areas in the whole group must have capacity']
        self.check_mode_p = validate_property(component, VC_STRING, 'CheckMode', default_val=check_mode_options[0],
                                              visible=True, step_values=check_mode_options)
        # self.queue_p = validate_property(component, 'List<Ref<Component>>', 'Queue', visible=False)
        self.queue_p = component.getProperty("Queue")  # Added (Pathway capacity group)
        self.areas = set()

    def add_area(self, area):
        self.areas.add(area)

    def has_capacity(self, path):
        # type: (Path) -> bool
        if 'route' in self.check_mode_p.Value:
            return all(a.has_capacity() for a in path.move_areas if a in self.areas)
        else:
            return all(a.has_capacity() for a in self.areas if a in self.areas)

    def place_on_queue(self, resource):
        # type: (vcComponent) -> None
        self.queue_p.Value += [resource]

    def remove_from_queue(self):
        # type: (vcComponent) -> None
        self.queue_p.Value = self.queue_p.Value[1:]

    def in_queue(self, component):
        # type: (vcComponent) -> bool
        if component:
            return component in self.queue_p.Value
        return False

    def next_in_queue(self, resource):
        # type: (vcComponent) -> bool
        return self.queue_p.Value[0] == resource

    def index_in_queue(self, resource):
        # type: (vcComponent) -> bool
        return self.queue_p.Value.index(resource)


class TravelController(object):
    PATH_SCHEMA_NAME = 'NavPaths'
    LINK_SCHEMA_NAME = 'NavLinks'

    CONFIG_AXIS_FWD_X_POS = '+x'
    CONFIG_AXIS_FWD_X_NEG = '-x'
    CONFIG_AXIS_FWD_BI = 'bi'
    CONFIG_CARRIAGE_DIFF = 'diff'
    CONFIG_CARRIAGE_OMNI = 'omni'

    def __init__(self, init_pos_mtx, tc_comp, update_signal):
        # type: (vcMatrix, vcComponent, vcStringSignal) -> None
        comp = getComponent()
        self._sim = getSimulation()

        self.nav_signal = tc_comp.findBehaviour('GenerateNavPath')  # type: vcComponentSignal
        self.transport_controller = tc_comp
        self.update_signal = update_signal
        self.tc_name = self.transport_controller.Name
        self._init_pos = vcMatrix.new(init_pos_mtx)  # used in reset

        self.position_matrix = vcMatrix.new(init_pos_mtx)
        self.paths = []
        self.areas = {'': None}
        self.area_groups = {'': None}
        self.sim = getSimulation()
        self.detector = self.sim.newCollisionDetector()
        self.active_path = None
        self.active_move = None
        self.default_travel_speed = 1000.0  # (mm/s)
        self.default_approach_speed = 300.0  # (mm/s)
        self.default_turn_speed = 30.0  # (deg/s)

        self.path_schema = None
        self.link_schema = None

        self.allow_reverse = True  # whether reversing in a single move (approach/exit) is allowed

        # threshold to determine if the resource stops in corners during travel (default 180° = always blend, no stop)
        self.turn_angle_threshold = validate_property(comp, VC_REAL, 'TurnInPlaceAngleThreshold', 180.0, visible=False,
                                                      unit='Angle').Value
        # max radius for blending
        self.turn_radius = validate_property(comp, VC_REAL, 'MaxTurnRadius', 800.0, visible=False,
                                             unit='Distance').Value

        # kinematics
        step_vals = [TravelController.CONFIG_AXIS_FWD_X_POS, TravelController.CONFIG_AXIS_FWD_X_NEG,
                     TravelController.CONFIG_AXIS_FWD_BI]
        self.travel_forward_axis = validate_property(comp, VC_STRING, 'TravelForwardAxis', step_vals[0], visible=False,
                                                     step_values=step_vals).Value
        step_vals = [TravelController.CONFIG_CARRIAGE_DIFF, TravelController.CONFIG_CARRIAGE_OMNI]
        self.carriage_type = validate_property(comp, VC_STRING, 'CarriageType', step_vals[0], visible=False,
                                               step_values=step_vals).Value

        # self.travel_forward_axis = config.kin_forward_axis # '+x', '-x', 'bi'
        # self.carriage_type = config.kin_carriage_type # 'diff, 'omni'

        # start_turn: if True, a sigle move or a path start by first aligning with the first line segement
        self.start_turn = self.carriage_type != TravelController.CONFIG_CARRIAGE_OMNI

        self.path_id_p = validate_property(comp, VC_INTEGER, 'Nav::PathId', visible=False)
        self.destination_p = validate_property(comp, VC_VECTOR, 'Nav::DestinationRequest', visible=False)
        self.hdg_p = validate_property(comp, VC_VECTOR, 'Nav::Heading', visible=False)
        self.cur_spd_p = validate_property(comp, VC_REAL, 'CurrentSpeed', visible=False)

        # Avoidance properties
        self.start_time_p = validate_property(comp, VC_REAL, 'Nav::StartTime', visible=False, unit='Time')
        self.path_times_p = validate_property(comp, VC_REAL, 'Nav::PathTimes', visible=False, step_values=[0.0])
        self.path_paused_p = validate_property(comp, VC_BOOLEAN, 'Nav::Paused', False, visible=False)
        self.rotating_p = validate_property(comp, VC_BOOLEAN, 'Nav::Rotating', False, visible=False)

        # self.pending_move = None # type: Move
        self.pending_move_target = None  # type: vcMatrix
        self.pending_move_state = ''
        self.first_travel = True

        self.total_travel_p = validate_property(comp, VC_REAL, 'TravelDistance')
        self.total_travel_p.Value = 0.0
        self.path_id_p.Value = -1
        self.start_time_p.Value = -1.0
        self.path_paused_p.Value = False

        self.visible_to_traffic = False
        self.current_area = None
        # self.next_area = None

        # Added (KIVA)
        if transport_raycast_show_raycast:
            self.raycast_sensor = comp.findBehaviour("RaycastSensor")
            self.detected_signal = comp.findBehaviour("DetectedSignal")
            self.range_signal = comp.findBehaviour("RangeSignal")
            self.product_lift = comp.getProperty("Transport::ProductLift")
            self.product_lift.Value = 0.0
            self.product_orientation = comp.getProperty("Transport::ProductOrientation")
            self.extra_lift_height = comp.getProperty("Transport::ExtraLiftHeight")
            self.lift_time = comp.getProperty("Transport::LiftTime")
            self.lock_product_orientation = comp.getProperty("Transport::LockProductOrientation")
            self.product_orientation_offset = 0.0

    def detect_lift_distance(self):

        self.detected_signal.Value = False
        self.range_signal.Value = 0.0
        self.raycast_sensor.Enabled = True
        delay(0.01)
        self.raycast_sensor.Enabled = False

    def move_lift(self, target, extra_lift):
        time = travel_controller.lift_time.Value

        if extra_lift:
            time = time * 0.2
        else:
            time = time * 0.8

        distance = target - self.product_lift.Value
        fps = 25
        steps = int(time * fps)
        time_step = time / steps
        distance_step = distance / steps
        for step_index in range(steps):
            travel_controller.product_lift.Value += distance_step
            delay(time_step)
        travel_controller.product_lift.Value = target

    def move_lift_interpolate(self, direction, rel_time):
        total_time = travel_controller.lift_time.Value
        extra_lift_time = total_time * 0.2
        lift_time = total_time * 0.8
        if direction == "unload":
            if rel_time <= extra_lift_time:
                travel_controller.product_lift.Value = travel_controller.range_signal.Value + (
                            1.0 - rel_time / extra_lift_time) * travel_controller.extra_lift_height.Value
            elif rel_time > extra_lift_time and rel_time <= total_time:
                travel_controller.product_lift.Value = (1.0 - (
                            rel_time - extra_lift_time) / lift_time) * travel_controller.range_signal.Value
                if config.cont.HeadComponent:
                    matrix = config.cont.HeadComponent.PositionMatrix
                    matrix.translateRel(0.0, 0.0, -matrix.P.Z - travel_controller.product_lift.Value)
                    config.cont.HeadComponent.PositionMatrix = matrix
        elif direction == "load":

            if rel_time <= lift_time:
                travel_controller.product_lift.Value = (rel_time / lift_time) * travel_controller.range_signal.Value
                if config.cont.HeadComponent:
                    matrix = config.cont.HeadComponent.PositionMatrix
                    matrix.translateRel(0.0, 0.0, -matrix.P.Z - travel_controller.product_lift.Value)
                    config.cont.HeadComponent.PositionMatrix = matrix
            elif rel_time > lift_time and rel_time <= total_time:
                travel_controller.product_lift.Value = travel_controller.range_signal.Value + (
                            (rel_time - lift_time) / extra_lift_time) * travel_controller.extra_lift_height.Value

    # Added (KIVA) - End

    def load_schemas(self):
        # call this from OnRun (let the TC construct the schema first)
        self.path_schema = self._get_schema(TravelController.PATH_SCHEMA_NAME)
        self.link_schema = self._get_schema(TravelController.LINK_SCHEMA_NAME)

    def load_areas(self):
        tc = self.transport_controller
        use_global_area_p = tc.getProperty('UseGlobalArea')
        pathways = tc.findBehaviour('Pathways').ConnectedComponents
        if pathways:
            for area in pathways:
                self.add_area(area)
                # Added (Pathway avoidance property)
                pathway_avoidance = area.getProperty("Avoidance")
                if not pathway_avoidance:
                    pathway_avoidance = area.createProperty(VC_BOOLEAN, "Avoidance")
                    pathway_avoidance.Value = True
        else:
            detect_obs = use_global_area_p.Value if use_global_area_p else False
            self.add_area(GlobalAreaComponent(detect_obstacles=detect_obs))

    def at_position(self, pos_m):

        # type: (vcMatrix) -> None
        self.interpolate(self._sim.SimTime)
        if self.position_matrix == pos_m:  # fast method (prone to rounding error)
            return True
        if 0.9998 < self.position_matrix.N * pos_m.N < 1.0002:  # 1 deg = 0.99984 / 1.000199
            p = self.position_matrix.P
            if abs(p.X - pos_m.P.X) < 1 and abs(p.Y - pos_m.P.Y) < 1 and abs(p.Z - pos_m.P.Z) < 1:
                return True
        return False

    def solve_move_heading(self, from_mtx, final_mtx, align_to_target=False):
        move_dir = final_mtx.P - from_mtx.P
        if comp.TurnSpeed <= 0:

            heading = comp.PositionMatrix.WPR.Z
        elif self.carriage_type == TravelController.CONFIG_CARRIAGE_OMNI:
            heading = self.solve_align_heading(final_mtx)
        elif move_dir.length() < 0.1:
            heading = self.solve_align_heading(final_mtx)
        else:
            if from_mtx.N * move_dir < 0 and self.allow_reverse and not align_to_target:
                move_dir = -1 * move_dir  # negate move_dir (reversing allowed)
            heading = math.atan2(move_dir.Y, move_dir.X) * 57.2957795  # align with move_dir
        return heading

    def solve_align_heading(self, final_mtx):
        if comp.TurnSpeed <= 0:
            return comp.PositionMatrix.WPR.Z
        if self.travel_forward_axis == TravelController.CONFIG_AXIS_FWD_BI:
            if self.position_matrix.N * final_mtx.N < 0:  # x_dot
                hdg = -1 * final_mtx.N
                return math.atan2(hdg.Y, hdg.X) * 57.2957795
        return final_mtx.WPR.Z

    def unsubscribe_listeners(self):
        for area in (a for a in self.areas.values() if a):
            area.used_capacity_p.OnChanged = None

    def reset(self):
        print("reset")
        self.position_matrix.P = self._init_pos.P
        self.position_matrix.setWPR(self._init_pos.WPR.X, self._init_pos.WPR.Y, self._init_pos.WPR.Z)
        self.first_travel = True
        self.active_move = None
        self.active_path = None
        self.total_travel_p.Value = 0.0
        for path in self.paths:  # type: Path
            path.reset_stats()

    def get_heading(self):
        # type: () -> vcVector
        if self.active_path:
            return vcVector.new(self.active_path.active_move.leg_vector)
        elif self.active_move:
            return vcVector.new(self.active_move.leg_vector)
        return vcVector.new()

    def get_hdg_matrix(self):
        # type: () -> vcMatrix
        m = vcMatrix.new()
        if self.active_path:
            m.N = self.active_path.active_move.leg_dir_vec
        elif self.active_move:
            m.N = self.active_move.leg_dir_vec
        else:
            m.N = self.position_matrix.N
        m.O = m.A ^ m.N
        # m.P = self.position_matrix.P
        return m

    def add_area(self, area_comp):
        if area_comp:
            area = NavArea(area_comp)
            self.areas[area_comp.Name] = area
            group = self._get_capacity_group(area_comp)
            area.assign_group(group)

    def add_area_group(self, group_comp):
        # type: (vcComponent) -> NavAreaCapacityGroup
        if group_comp:
            name = group_comp.Name
            if name in self.area_groups.keys():
                return self.area_groups[name]
            else:
                group = NavAreaCapacityGroup(group_comp)
                self.area_groups[name] = group
                return group
        return None

    def _get_capacity_group(self, area_comp):
        # type: (vcComponent) -> None
        group_ife = area_comp.findBehaviour('CapacityGroup')  # type: vcSimInterface
        if group_ife:
            return self.add_area_group(group_ife.ConnectedComponent)
        return None

    def get_path(self, from_vec, to_vec, velocity):
        # type: (vcVector, vcVector, float) -> Path
        path = self._find_path_from_store(from_vec, to_vec)  # type: Path
        if path and comp.FindPathFromStore:  # Added (Elevator)
            path.reset()
            path.set_speeds(velocity, velocity)
            path.velocity_setting = velocity
            return path

        path_ref = self.generate_path_ref(from_vec, to_vec)
        path_data = self._path_in_schema(path_ref)
        if not path_data:
            self.path_id_p.Value = -1
            self.start_time_p.Value = -1.0
            self.destination_p.Value = to_vec
            self.nav_signal.signal(comp)
            triggerCondition(lambda: self.update_signal.Value == 'nav')
            path_data = self._path_in_schema(self.path_id_p.Value)  # set by TC
        if path_data:
            path = travel_controller._create_path_from_schema(path_data, velocity)
        else:
            # print warning('Path not found!')
            # Create a straight "dummy" path
            path = self.create_path(-1, '', [from_vec, to_vec], velocity)

        # Added (Elevator)
        if not comp.FindPathFromStore:
            self.destination_p.Value = to_vec

        return path

    def generate_path_ref(self, from_vec, to_vec):
        # type: (vcVector, vcVector) -> str
        return '{},{},{};{},{},{}'.format(
            int(from_vec.X / 10.0),
            int(from_vec.Y / 10.0),
            int(from_vec.Z / 10.0),
            int(to_vec.X / 10.0),
            int(to_vec.Y / 10.0),
            int(to_vec.Z / 10.0)
        )

    def create_path(self, path_id, path_ref, target_vectors, velocity=0.0, areas=[]):
        # type: (int, str, List[vcVector], float, List[NavArea]) -> Path
        # if len(target_vectors) < 2:
        #   raise ValueError('TravelController.create_path: "target_vectors" must have at least two members. For a single move, construct "Move" instance.')
        if areas and len(areas) != len(target_vectors):
            raise ValueError('TravelController.create_path: ERROR: "areas" and "target_vectors" length mismatch!')

        if not velocity:
            velocity = self.default_travel_speed

        if not path_ref:
            from_vec = target_vectors[0]
            to_vec = target_vectors[-1]
            path_ref = self.generate_path_ref(from_vec, to_vec)

        if len(target_vectors) > 1 and (self.position_matrix.P - target_vectors[0]).length() < 1.0:
            target_vectors.pop(0)  # skip first vector (equal to initial pos)

        path = Path(self, self.position_matrix, path_id, path_ref, self.turn_angle_threshold, self.turn_radius)
        start_turn = self.start_turn
        for i, target_vec in enumerate(target_vectors):
            area = None if not areas else areas[i]  # type: NavArea
            path.add_target(
                target_vec,
                velocity,
                self.default_turn_speed,
                start_turn,
                area=area
            )
            start_turn = False
        path.velocity_setting = velocity
        path.current_velocity_setting = velocity
        path.unique_areas = set(path.move_areas)
        self.paths.append(path)
        return path

    def single_move(self, to_pos_vector, velocity, turn_velocity=None, heading=None, turn_in_place=True,
                    move_type=1):  # Added (KIVA) - last argument
        # type: (vcVector, float, bool) -> Move
        self.interpolate(self.sim.SimTime)  # update position
        if not turn_velocity:
            turn_velocity = self.default_turn_speed
        angle_threshold = self.turn_angle_threshold if not turn_in_place else 0.0
        if heading is None:
            heading = Path.get_vector_heading(self.position_matrix.P, to_pos_vector)
        move = Move(self.sim.SimTime, self.position_matrix, to_pos_vector, heading, velocity, turn_velocity,
                    angle_threshold, self.turn_radius, move_type)  # Added (KIVA) - last argument
        return move

    def activate_move(self, move):
        # type: (Move) -> None
        self.update_total_travel()
        self.active_path = None
        self.path_id_p.Value = 0 if self.visible_to_traffic else -1
        self.path_paused_p.Value = True
        self.start_time_p.Value = self.sim.SimTime
        move.set_start_time(self.sim.SimTime)
        self.active_move = move

    def activate_path(self, path):
        # type: (Path) -> None
        self.update_total_travel()
        if self.travel_forward_axis == TravelController.CONFIG_AXIS_FWD_BI:
            self.interpolate(self.sim.SimTime)
            path.re_evaluate_hdg(self.sim.SimTime)
        self.active_path = path  # type: Path
        self.path_id_p.Value = path.id
        path.start(self.sim.SimTime)

    def deactivate(self):
        self.update_total_travel()
        self.active_path = None
        self.active_move = None
        self.path_id_p.Value = 0 if self.visible_to_traffic else -1
        self.start_time_p.Value = -1.0
        self.rotating_p.Value = False

    def interpolate(self, sim_time):
        if self.active_path and self.active_path.active_move:
            emergency_stop_gui(True)
            self.active_path.interpolate(sim_time)

            # Added (Emergency Stop)
            if manual_pause:
                if comp.EmergencyStop and not self.active_path.paused:
                    self.active_path.pause()
                    stats_manager.set_state('Blocked')
                elif not comp.EmergencyStop and self.active_path.paused:
                    self.active_path.resume()
                    stats_manager.set_state(halt_state)

            active_m = self.active_path.active_move
            self.cur_spd_p.Value = active_m.velocity if active_m and not self.active_path.paused and not active_m.rotating else 0.0
            self.rotating_p.Value = bool(active_m and active_m.rotating and not self.active_path.paused)
            if active_m:
                self.hdg_p.Value = active_m.leg_dir_vec
            else:
                self.deactivate()  # end of path
        elif self.active_move:
            emergency_stop_gui(False)
            self.active_move.interpolate(sim_time)
            if self.sim.SimTime > self.active_move.end_time and not self.active_move.paused:
                self.deactivate()  # end of move
                self.cur_spd_p.Value = 0.0
            else:
                self.cur_spd_p.Value = self.active_move.velocity if not self.active_move.rotating else 0.0
                self.hdg_p.Value = self.active_move.leg_dir_vec
                self.rotating_p.Value = self.active_move.rotating and not self.active_move.paused

        # Added (KIVA)
        if transport_raycast_show_raycast:
            if travel_controller.lock_product_orientation.Value != "None":
                travel_controller.product_orientation.Value = self.position_matrix.WPR.Z

        parent_matrix = comp.Parent.WorldPositionMatrix  # Added (Sub-layout)
        parent_matrix.invert()  # Added (Sub-layout)
        return parent_matrix * self.position_matrix

    def update_total_travel(self):
        travel = 0.0
        path = self.active_path
        move = self.active_move
        if path:
            if path.active_move == path.moves[-1] or path.active_move is None:
                travel = path.length
            else:
                for move in path.moves:
                    travel += move.length
                    if move == path.active_move:
                        break
        elif move:
            travel = move.length
        self.total_travel_p.Value += travel

    def dump_stats(self):
        all_moves = [m for p in self.paths for m in p.moves if
                     m.stats_travel_count or m.stats_collision_count]  # flat list
        if not self.link_schema or not all_moves:
            return

        for i in range(self.link_schema.DataCount):
            link_data = self.link_schema.getDataValues(i)
            from_vec = link_data[PathLinkSchema.FROM]
            to_vec = link_data[PathLinkSchema.TO]
            travel_stats = link_data[PathLinkSchema.TRAVEL_STATS]
            avoid_stats = link_data[PathLinkSchema.AVOIDANCE_STATS]
            for move in all_moves:  # type: Move
                move_from_vec = vcVector.new(move.init_pos_x, move.init_pos_y, move.init_pos_z)
                if (move.target_vec - to_vec).length() < 1.0 and (move_from_vec - from_vec).length() < 1.0:
                    self.link_schema.setDataValue(i, PathLinkSchema.TRAVEL_STATS,
                                                  travel_stats + move.stats_travel_count)
                    self.link_schema.setDataValue(i, PathLinkSchema.AVOIDANCE_STATS,
                                                  avoid_stats + move.stats_collision_count)

    def _get_schema(self, name):
        schema = app.findLayoutItem(name)
        if not schema:
            raise Exception('Schema "{}" not found!'.format(name))
        return schema

    def _path_in_schema(self, path_id_or_ref):
        # type: (...) -> Tuple[...]
        # Get path from schema based on ref ir id
        data_column = PathSchema.REF if type(path_id_or_ref) is str else PathSchema.ID
        for i in range(self.path_schema.DataCount):
            if self.path_schema.getDataValue(i,
                                             data_column) == path_id_or_ref:  # and self.path_schema.getDataValue(i, PathSchema.CONTROLLER) == self.tc_name:
                return self.path_schema.getDataValues(i)
        return None

    def _find_path_from_store(self, from_vec, to_vec, path_ref=''):
        # type: (vcVector, vcVector, Optional[str]) -> Optional[Path]
        for path in self.paths:
            # if path.id == path_id or (path.from_vec == from_vec and path.to_vec == to_vec):
            if path.ref == path_ref or (
                    (path.from_vec - from_vec).length() < 1.0 and (path.to_vec - to_vec).length() < 1.0):
                return path
        return None

    def _create_path_from_schema(self, path_data, velocity):
        move_ids = [int(i) for i in path_data[PathSchema.LINKS].split(',')]
        move_data = [None] * len(move_ids)  # type: List[Tuple(...)]
        for i in range(self.link_schema.DataCount):
            move_id = self.link_schema.getDataValue(i, PathLinkSchema.ID)
            if move_id in move_ids:
                move_data[move_ids.index(move_id)] = self.link_schema.getDataValues(i)

        if not all(move_data):
            raise Exception('{} - ERROR: Incomplete moves for path "{}"!'.format(comp.Name, path_data[1]))

        target_vecs = [m[PathLinkSchema.TO] for m in move_data]
        areas = [self.areas[m[PathLinkSchema.AREA]] for m in move_data]  # type: List[NavArea]

        path = self.create_path(path_data[PathSchema.ID], path_data[PathSchema.REF], target_vecs, velocity, areas=areas)
        return path


class Path(object):
    def __init__(self, controller, mtx, path_id, path_ref, angle_threshold=180.0, turn_radius=800.0):
        self.position_matrix = mtx  # Initial values. Matrix will be manipulated by the moves (interpolators)
        self.orientation_vec = vcVector.new()
        self.id = path_id
        self.ref = path_ref
        self.moves = []  # type: List[Move]
        self.move_areas = []  # type: List[NavArea]
        self.controller = controller
        self.travel_time = 0.0
        self.active_move = None  # type: Move
        self.turn_angle_threshold = angle_threshold
        self.turn_radius = turn_radius
        self.from_vec = None
        self.to_vec = None
        self.velocity_setting = 0.0  # original velocity setting
        self.current_velocity_setting = 0.0
        self._last_stats_update = -1.0
        # self.prev_area = None # type: NavArea
        self.next_area = None  # type: NavArea
        self.paused = False
        self.pause_start_time = 0.0
        self.length = 0.0  # total lenght of the path
        self.unique_areas = set()
        self.flipped = False

        self.area_traverse_index = 0

    def reset(self):
        for move in self.moves:
            move.reset()

    def start(self, sim_time):
        # self.prev_area = None
        # self.next_area = None
        self.controller.path_paused_p.Value = False
        self.paused = False
        self.area_traverse_index = 0
        shift_time = -1.0
        for move in self.moves:
            # move.reset()
            if shift_time == -1.0:  # first move
                shift_time = move.set_start_time(sim_time)
            elif shift_time > 0:
                move.shift(shift_time)

        self.active_move = self.moves[0]
        # AVOIDANCE
        self.controller.start_time_p.Value = sim_time
        self.controller.path_times_p.StepValues = self.get_times()

    def pause(self):
        if self.paused:
            return
        self.interpolate(sim.SimTime)
        self.paused = True
        self.pause_start_time = sim.SimTime
        self.controller.hdg_p.Value = self.active_move.leg_vector
        # AVOIDANCE
        self.controller.path_paused_p.Value = True

    def resume(self):
        if not self.paused:
            return
        shift_time = sim.SimTime - self.pause_start_time
        for move in self.moves:
            move.shift(shift_time)
        self.paused = False
        # AVOIDANCE
        self.controller.path_paused_p.Value = False
        self.controller.start_time_p.Value = self.moves[0].start_time
        self.controller.path_times_p.StepValues = self.get_times()

    def add_target(self, to_pos_vector, velocity, turn_velocity, turn_in_place, heading=None, area=None, blending=True):
        # type: (vcVector, float, float, bool, Optional[float], Optional[NavArea], Optional[bool]) -> Move
        if self.moves:
            prev_move = self.moves[-1]  # type: Move
            # gather initial values for the next move
            start_time = prev_move.end_time
            prev_move.interpolate(start_time)  # writes to self.position_matrix
        else:
            prev_move = None
            start_time = self.controller.sim.SimTime

        if heading is None:
            if turn_velocity <= 0:
                heading = self.position_matrix.WPR.Z
            else:
                heading = Path.get_vector_heading(self.position_matrix.P, to_pos_vector)  # align with the segment
                # flip if necessary, define flip only on first move
                if not self.moves:
                    if self.controller.travel_forward_axis == TravelController.CONFIG_AXIS_FWD_X_NEG:
                        flip = True
                    elif self.controller.travel_forward_axis == TravelController.CONFIG_AXIS_FWD_BI:
                        rot = heading - self.position_matrix.WPR.Z
                        if rot > 180.0:
                            rot -= 360.0
                        elif rot < -180.0:
                            rot += 360.0
                        flip = abs(rot) > 90
                    else:
                        flip = False
                    self.flipped = flip

        if self.flipped:
            if heading <= 0:
                heading += 180.0
            else:
                heading -= 180.0
        print
        "zalupa999"
        if area and area.limited_speed_p.Value:
            print
            "zalupa666"
            velocity = velocity if area.speed_limit_p.Value >= velocity else area.speed_limit_p.Value

        angle_threshold = self.turn_angle_threshold if not turn_in_place else 0.0
        move = Move(start_time, self.position_matrix, to_pos_vector, heading, velocity, turn_velocity, angle_threshold,
                    self.turn_radius, 2)  # Added (KIVA) - Last argument

        if blending and prev_move and not move.turn_in_place:
            move.blend_with(prev_move)
        self.travel_time += move.duration
        self.length += move.length
        self.moves.append(move)
        self.move_areas.append(area)

        if self.from_vec is None:
            self.from_vec = vcVector.new(self.position_matrix.P)
        self.to_vec = move.target_vec

        return move

    def update_stats(self):
        sim_time = self.controller.sim.SimTime
        for move in (m for m in self.moves if self._last_stats_update < m.start_time <= sim_time):  # type: Move
            move.stats_travel_count += 1
        self._last_stats_update = sim_time

    def reset_stats(self):
        for move in self.moves:
            move.stats_travel_count = 0
        self._last_stats_update = -1.0

    def re_evaluate_hdg(self, sim_time):
        ''' in case of bi-directional movement. Check headings '''
        rot = self.moves[0].init_rot - self.position_matrix.WPR.Z
        if rot > 180.0:
            rot -= 360.0
        elif rot < -180.0:
            rot += 360.0
        if abs(rot) > 90:  # flip
            map(lambda m: m.flip_rotation(), self.moves)
            map(lambda m: m.recalc(sim_time), self.moves)

    def get_last_pos_target(self):
        # type: () -> vcMatrix
        if self.moves:
            return self.interpolate(self.moves[-1].end_time)
        else:
            return self.position_matrix

    def set_speeds(self, travel, approach):
        # if the path is active, loop trough incompleted interpolators and update
        # otherwise calculate new velocities for all moves

        self.current_velocity_setting = travel
        if travel <= 0:
            self.pause()
            return
        self.resume()  # if paused

        time_shift = 0.0
        t = self.controller.sim.SimTime if self.controller.active_path == self else self.moves[0].start_time
        for i, move in enumerate(self.moves):  # type: int, Move
            if t > move.end_time:
                continue
            if move.type == Move.APPROACH:  # not used
                move.velocity = approach
            else:
                move.velocity = travel

            area = self.move_areas[i]  # type: NavArea
            if area and area.limited_speed_p.Value and move.velocity > area.speed_limit_p.Value:
                move.velocity = area.speed_limit_p.Value

            time_shift += move.recalc(t)

            if move.start_time > t:
                move.set_start_time(self.moves[i - 1].end_time)

        self.travel_time += time_shift
        # AVOIDANCE
        self.controller.start_time_p.Value = self.moves[0].start_time
        self.controller.path_times_p.StepValues = self.get_times()

    def set_move_speed(self, move, travel, approach):

        # type: (Move, float, float) -> None
        if move.type == Move.TRAVEL and move.velocity != travel:
            move.velocity = travel
        elif move.type == Move.APPROACH and move.velocity != approach:  # not used
            move.velocity = approach
        else:
            return
        time_shift = move.recalc(move.start_time)
        for other_move in (m for m in self.moves if m.start_time > move.start_time):
            other_move.shift(time_shift)
        self.travel_time += time_shift

    def set_current_move_speed(self, travel, approach):
        time_shift = 0.0
        print
        'Zalupa'
        for move in self.moves:  # type: Move
            # current (active) interpolator
            if move.start_time <= self.controller.sim.SimTime < move.end_time:
                if move.type == Move.APPROACH:
                    move.velocity = approach
                else:
                    move.velocity = travel
                time_shift = move.recalc(self.controller.sim.SimTime)
            elif time_shift > 0:
                move.shift(time_shift)
        self.travel_time += time_shift

    def get_time_to_target(self, sim_time):
        return self.moves[-1].end_time - sim_time if not self.paused else 1e8

    def get_time_to_next_area2(self):
        # returns 1e8 seconds (~3 years) if no area crossings, otherwise relative to to next area crossing
        i = self.area_traverse_index

        if self.paused:
            return 1e8

        # print i, self.move_areas
        if i == 0 and self.move_areas[0] is None and len(self.unique_areas) > 1:
            # coming outside of an area, start moving only after the area has capacity
            # doesn't work if only one link in the path (no common area in the link)
            self.next_area = self.move_areas[1]
            self.area_traverse_index = 1
            self.controller.first_travel = False
            return 0.0

        if self.controller.first_travel:
            # starting sim on top of an area
            self.next_area = self.move_areas[0]
            self.controller.first_travel = False
            return 0.0

        if i == 0 and not self.controller.current_area and self.move_areas[0]:
            # came from outside, first move inside area
            self.next_area = self.move_areas[0]
            return 0.0

        if len(self.unique_areas) == 1 and self.controller.current_area != None:
            return 1e8  # only one area, no crossings

        # print 'Move areas:', self.move_areas

        sim_time = sim.SimTime
        while i < len(self.move_areas):
            # print i, 'current:', self.controller.current_area, 'traverse next', self.move_areas[i]
            if self.controller.current_area != self.move_areas[i]:
                move = self.moves[i - 1] if i > 0 else self.moves[0]
                self.next_area = self.move_areas[i]
                # print 'NEXT AREA', self.next_area, 'time', move.end_time - sim_time
                self.area_traverse_index = i
                return (move.end_time - 0.001) - sim_time
            i += 1
        return 1e8

    # def get_time_to_next_area(self):
    #   # type: (float) -> float
    #   pairs = ((0, 0),) if len(self.move_areas) == 1 else enumerate(range(1, len(self.move_areas)))
    #   for prev_i, next_i in pairs:
    #     move = self.moves[prev_i]
    #     if prev_i == 0 and self.move_areas[0] is None:
    #       # coming outside of an area, start moving only after the area has capacity
    #       self.next_area = self.move_areas[next_i]
    #       self.controller.first_travel = False
    #       yield 0.0
    #     elif self.controller.first_travel:
    #       # starting sim on top of an area
    #       self.next_area = self.move_areas[prev_i]
    #       self.controller.first_travel = False
    #       yield 0.0
    #     elif self.move_areas[prev_i] != self.move_areas[next_i] and sim.SimTime < move.end_time:
    #       self.next_area = self.move_areas[next_i]
    #       self.prev_area = self.move_areas[prev_i]
    #       yield move.end_time - sim.SimTime
    #   yield self.travel_time # no area crossings

    def interpolate(self, sim_time):
        # if self.active_move and sim_time <= self.active_move.end_time:
        #   return self.active_move.interpolate(sim_time)
        if self.paused and self.pause_start_time <= sim_time:
            return self.position_matrix
        for move in self.moves:  # type: Move
            if sim_time <= move.end_time:
                self.active_move = move
                break
            else:
                self.active_move = None  # end of path
        return move.interpolate(sim_time)

    def get_path_data(self):
        # travel_velos = []
        turn_times = []
        times = []  # [self.moves[0].start_time]
        for move in self.moves:  # type: Move
            # travel_velos.append(move.velocity)
            turn_times.append(move.rotation_time if move.turn_in_place else 0)
            times.append(move.end_time)
        return times, turn_times

    def get_times(self):
        times = []
        for move in self.moves:  # type: Move
            # if self.pause and move.end_time < self.pause_start_time:
            #   # fill following segments with zeros
            #   times.append(0.0)
            #   times.append(0.0)
            # else:
            if move.turn_in_place:
                times.append(move.start_time + move.rotation_time)
            else:
                times.append(move.start_time)
            times.append(move.end_time)
        return times

    @staticmethod
    def get_vector_heading(from_vector, to_vector):
        # returns the angle (in degrees) of a vector between from_vector and to_vector
        target_vector = to_vector - from_vector
        return math.atan2(target_vector.Y, target_vector.X) * 57.2957795

    @staticmethod
    def get_relative_heading(pos_matrix, target_vector):
        target_heading = target_vector - pos_matrix.P
        angle = math.atan2(target_heading.Y, target_heading.X) * 57.2957795 - pos_matrix.WPR.Z
        if angle > 180:
            angle -= 360
        return angle


class Move(object):
    APPROACH = 1
    TRAVEL = 2

    def __init__(self, start_time, pos_mtx, to_pos_vector, to_rot, velocity, turn_velocity,
                 turn_in_place_angle_threshold, turn_radius, move_type):
        # pos_mtx: is used to set the initial position and orientation values
        # to_rot: WPR.Z in World coordinates
        self.start_time = start_time
        self.start_time_offset = 0.0  # relative time to the start of translational movement
        self.duration = 0.0
        self.rotation_time = 0.0
        self.rotating = False  # True if rotating in place
        self.paused = False
        self.pause_start_time = 0.0

        self.pos_mtx = pos_mtx  # used to write pos/orientation vectors to
        self.pos_vec = vcVector.new()  # used to write pos
        self.orient_vec = vcVector.new()  # used to write orientation

        self.turn_radius = turn_radius  # max turn radius for blending

        self.init_pos_x = pos_mtx.P.X
        self.init_pos_y = pos_mtx.P.Y
        self.init_pos_z = pos_mtx.P.Z
        self.init_rot = pos_mtx.WPR.Z

        self.init_pos_x_offset = 0.0  # used for speed change updates
        self.init_pos_y_offset = 0.0  # used for speed change updates
        self.init_pos_z_offset = 0.0  # used for speed change updates
        self.target_vec = to_pos_vector
        self.target_rotation = to_rot
        self.velocity = velocity
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.turn_velocity = turn_velocity
        self.type = move_type

        # self.blending = False
        self.next_move = None
        self.blend_in_dist = 0.0
        self.blend_out_dist = 0.0
        self.blend_in_radius = 0.0
        self.blend_out_radius = 0.0
        self.blend_in_ang_velocity = 0.0
        self.blend_in_z_velocity = 0.0
        self.blend_out_ang_velocity = 0.0
        self.blend_out_z_velocity = 0.0
        self.blend_in_end_time_offset = -1.0
        self.blend_out_start_time_offset = -1.0
        self.blend_out_pos = None  # type: vcVector
        # self.blend_in_climb_angle = 0.0
        # self.blend_out_climb_angle = 0.0
        self.blend_in_theta_offset = 0.0
        self.blend_out_theta_offset = 0.0

        self.leg_rotation = self.target_rotation - self.init_rot
        if self.leg_rotation > 180:
            self.leg_rotation -= 360
        elif self.leg_rotation < -180:
            self.leg_rotation += 360

        self.turn_in_place = False if abs(self.leg_rotation) < turn_in_place_angle_threshold else True

        self.leg_vector = to_pos_vector - pos_mtx.P  # type: vcVector
        self.length = self.leg_vector.length()
        if self.length > 0:
            self.leg_dir = math.degrees(math.atan2(self.leg_vector.Y, self.leg_vector.X))
            self.leg_dir_vec = vcVector.new(self.leg_vector.X, self.leg_vector.Y, 0.0)
            self.leg_dir_vec.normalize()
        else:
            self.leg_dir = to_rot
            self.leg_dir_vec = vcVector.new(math.cos(math.radians(to_rot)), math.sin(math.radians(to_rot)), 0.0)
        self.rot_dir = -1 if self.leg_rotation < 0 else 1
        self.end_time = 0.0

        self.stats_travel_count = 0
        self.stats_collision_count = 0

        # Added (KIVA)
        self.unload_time = 0.0
        self.load_time = 0.0

        # Added (KIVA)
        if transport_raycast_show_raycast:
            if travel_controller.lock_product_orientation.Value == "Ground":
                if travel_controller.first_travel:
                    self.turn_in_place = True
                elif config.cont.HeadComponent:
                    self.turn_in_place = True
                else:
                    self.turn_in_place = False

        self._calculate_times()

    def flip_rotation(self):

        # do recal() after flipping
        if self.init_rot <= 0:
            self.init_rot += 180.0
        else:
            self.init_rot -= 180.0
        if self.target_rotation <= 0:
            self.target_rotation += 180.0
        else:
            self.target_rotation -= 180.0
        self.leg_rotation = self.target_rotation - self.init_rot
        if self.leg_rotation > 180:
            self.leg_rotation -= 360
        elif self.leg_rotation < -180:
            self.leg_rotation += 360
        self.rot_dir = -1 if self.leg_rotation < 0 else 1
        if self.blend_in_radius:
            self.blend_in_radius *= -1
        if self.blend_out_radius:
            self.blend_out_radius *= -1

    def _calc_blend_in(self, length=-1, sim_time=0.0):
        dist = self.blend_in_dist if length < 0 else length
        blend_duration = dist / self.velocity

        rotation = self.leg_rotation / 2.0
        init_rot = 0.0
        if self.start_time_offset > 0.0:  # self.blend_in_ang_velocity and sim_time > 0.0:
            init_rot = self.blend_in_ang_velocity * self.start_time_offset + self.blend_in_theta_offset
            rotation = -init_rot

        self.blend_in_ang_velocity = rotation / blend_duration
        self.blend_in_end_time_offset = self.start_time_offset + blend_duration
        offset = init_rot - self.blend_in_ang_velocity * self.start_time_offset

        if init_rot:
            self.blend_in_theta_offset = offset
        else:
            self.blend_in_theta_offset = -rotation

        climb_angle_rad = math.sin(self.leg_vector.Z / self.length)
        self.blend_in_z_velocity = dist * math.tan(climb_angle_rad) / blend_duration

    def _calc_blend_out(self, length_left=-1):
        # print 'CALC LENGTH LEFT:', length_left
        offset = 0.0
        rotation = self.next_move.leg_rotation / 2.0
        if length_left < 0 or self.blend_out_dist <= length_left:
            blend_duration = self.blend_out_dist / self.velocity
            self.blend_out_start_time_offset = self.duration - blend_duration
            climb_angle_rad = math.sin(self.leg_vector.Z / self.length)
            self.blend_out_z_velocity = self.blend_out_dist * math.tan(climb_angle_rad) / blend_duration
        else:
            blend_duration = length_left / self.velocity
            init_time = self.start_time_offset - self.blend_out_start_time_offset  # sim.SimTime - self.start_time - self.blend_out_start_time_offset
            init_rot = self.blend_out_ang_velocity * init_time + self.blend_out_theta_offset
            rotation -= init_rot
            new__ang_velo = rotation / blend_duration
            new_rot = new__ang_velo * init_time
            offset = init_rot - new_rot
            # print 'MID BLEND: duration', blend_duration, 'init time', init_time, 'init rot', init_rot, 'full rot', rotation

        self.blend_out_theta_offset = offset

        if self.rotation_time + blend_duration > self.transition_time:
            # print ' UNABLE TO ROTATE IN TIME', sim.SimTime

            # solving new velocity from: self.start_time_offset + length/self.velocity = self.rotation_time + blend_duration
            # A = self.start_time_offset
            # L = self.length
            # D = self.rot_dir*self.leg_rotation
            # V = self.turn_velocity
            # B = self.blend_out_dist
            # x = self.velocity
            # solve x: A + L/x = D/V + B/x
            # x = (V (B - L))/(A V - D) and D!=A V and B V!=L V

            init_end_time = self.end_time
            self.velocity = (self.turn_velocity * (self.blend_out_dist - self.length)) / (
                        self.start_time_offset * self.turn_velocity - self.rot_dir * self.leg_rotation)
            self.transition_time = self.start_time_offset + self.length / self.velocity
            self.duration = self.start_time_offset + self.length / self.velocity
            self.velocity_x = self.leg_vector.X / self.length * self.velocity
            self.velocity_y = self.leg_vector.Y / self.length * self.velocity
            self.velocity_z = self.leg_vector.Z / self.length * self.velocity
            blend_duration = self.blend_out_dist / self.velocity
            self.end_time = self.start_time + self.duration
            # shift next move
            self.next_move.shift(self.end_time - init_end_time)

        self.blend_out_ang_velocity = rotation / blend_duration
        # print 'BLEND VELOCITY', self.blend_out_ang_velocity, 'rotation', rotation

        if not self.blend_out_pos:
            t = self.blend_out_start_time_offset - self.start_time_offset
            self.blend_out_pos = vcVector.new(
                self.init_pos_x + self.velocity_x * t,
                self.init_pos_y + self.velocity_y * t,
                self.init_pos_z + self.velocity_z * t
            )

    def blend_with(self, prev_move):
        alpha_rad = 0.0174532 * (180.0 - abs(self.leg_rotation)) / 2.0  # abs. circle tangent line angle (slope)
        if -1.0 > self.leg_vector.Z > 1.0:  # has climb (< -1 or > 1.0)
            xy_dist_1 = math.sqrt(self.leg_vector.X ** 2 + self.leg_vector.Y ** 2)
            xy_dist_2 = math.sqrt(prev_move.leg_vector.X ** 2 + prev_move.leg_vector.Y ** 2)
            blend_dist = min(self.turn_radius / math.tan(alpha_rad), xy_dist_1 / 2.0, xy_dist_2 / 2.0)
        else:
            blend_dist = min(self.turn_radius / math.tan(alpha_rad), self.length / 2.0, prev_move.length / 2.0)

        if blend_dist < 1.0:
            return

        self.blend_in_dist = blend_dist
        # self.blend_in_climb_angle = math.sin(self.leg_vector.Z/self.length)*57.2957795
        self._calc_blend_in()

        self.blend_in_radius = blend_dist * math.tan(alpha_rad)
        if self.blend_in_ang_velocity > 0:
            self.blend_in_radius *= -1

        flip = abs(self.target_rotation - self.leg_dir) > 90.0
        if flip:
            self.blend_in_radius *= -1

        prev_move.blend_out_dist = self.blend_in_dist
        prev_move.blend_out_radius = self.blend_in_radius
        prev_move.next_move = self
        # prev_move.blend_out_climb_angle = math.sin(prev_move.leg_vector.Z/prev_move.length)*57.2957795
        # print 'ANGLE: blend_out_climb_angle', prev_move.blend_out_climb_angle
        prev_move._calc_blend_out()

        '''
    if abs(self.blend_in_ang_velocity) > self.turn_velocity:
      print '*** TURN VELOCITY EXCEEDED (blend in) ***'
    if abs(prev_move.blend_out_ang_velocity) > self.turn_velocity:
      print '*** TURN VELOCITY EXCEEDED (blend out) ***'
    '''

        # disable default start rotation and update duration
        self.turn_in_place = False
        self.rotation_time = 0.0
        self.duration = self.transition_time
        self.end_time = self.start_time + self.transition_time

    def _calculate_times(self, sim_time=0.0, length=-1):
        length = self.length if length < 0 else length

        # print 'start:', self.start_time, 'blend in radi', self.blend_in_radius, 'start offset', self.start_time_offset
        if self.blend_in_radius == 0:  # without blending
            self.rotation_time = self.rot_dir * self.leg_rotation / self.turn_velocity if self.turn_velocity > 0 else 0.0
            # print 'without blending rotation_time::', self.rotation_time, 'angle:', self.leg_rotation, 'velo', self.turn_velocity
            if self.turn_in_place and self.start_time_offset < self.rotation_time:
                # rotation not completed yet. Match start time with rotation end time
                self.start_time_offset = self.rotation_time

                # Added (KIVA)
            if transport_raycast_show_raycast:
                if travel_controller.lock_product_orientation.Value == "Ground" and config.cont.HeadComponent and self.type == 2:
                    self.unload_time = travel_controller.lift_time.Value
                    self.load_time = travel_controller.lift_time.Value
                    if self.turn_in_place and self.start_time_offset < self.rotation_time + self.unload_time + self.load_time:
                        self.start_time_offset = self.rotation_time + self.unload_time + self.load_time

        self.transition_time = self.start_time_offset + length / self.velocity
        # print 'start offset', self.start_time_offset
        # print 'turn_in_place', self.turn_in_place, 'transition_time', self.transition_time, 'rotation_time', self.rotation_time

        if self.turn_in_place:
            self.duration = self.transition_time
        else:
            self.duration = self.transition_time if self.transition_time > self.rotation_time else self.rotation_time

        if self.length:
            self.velocity_x = self.leg_vector.X / self.length * self.velocity
            self.velocity_y = self.leg_vector.Y / self.length * self.velocity
            self.velocity_z = self.leg_vector.Z / self.length * self.velocity
        self.velocity_rot = self.rot_dir * self.turn_velocity
        self.end_time = self.start_time + self.duration

        # recalc blends
        if self.blend_in_radius and self.start_time_offset < self.blend_in_end_time_offset:
            self._calc_blend_in(self.blend_in_dist - (self.length - length), sim_time)
        if self.blend_out_radius:
            self._calc_blend_out(length)

        # print 'times - start:', self.start_time, ', end:', self.end_time
        # print 'Rotation time:', self.rotation_time

    def recalc(self, sim_time):
        ''' Call after altering speeds '''
        # print 're-calculating....'
        old_end_time = self.end_time

        if self.start_time < sim_time < self.end_time:
            # print '***MID-MOVE**** duration:', self.duration, 'VEL:', self.velocity
            self.interpolate(sim_time, blending=False)
            self.init_pos_x_offset = self.pos_mtx.P.X - self.init_pos_x
            self.init_pos_y_offset = self.pos_mtx.P.Y - self.init_pos_y
            self.init_pos_z_offset = self.pos_mtx.P.Z - self.init_pos_z
            self.start_time_offset = sim_time - self.start_time
            # print 'start_time_offset', self.start_time_offset, 'xyz', self.init_pos_x_offset, self.init_pos_y_offset, self.init_pos_z_offset
            if self.init_pos_x_offset != 0 or self.init_pos_y_offset != 0 or self.init_pos_z_offset != 0:
                # length_left = (self.target_vec - self.pos_mtx.P).length()
                length_left = math.sqrt(
                    (self.target_vec.X - self.pos_mtx.P.X) ** 2 + (self.target_vec.Y - self.pos_mtx.P.Y) ** 2 + (
                                self.target_vec.Z - self.pos_mtx.P.Z) ** 2)  # travel leftovers
            else:
                length_left = self.length
            # print 'lenght left', length_left
            self._calculate_times(sim_time, length_left)
        elif sim_time != self.end_time:  # if sim_time == end_time, do nothing
            self.reset()
            self._calculate_times(sim_time)

        # print 'COMPLETE: old end', old_end_time, 'new end', self.end_time, 'shift', self.end_time - old_end_time
        return self.end_time - old_end_time

    def reset(self):
        self.init_pos_x_offset = 0.0
        self.init_pos_y_offset = 0.0
        self.blend_in_theta_offset = 0.0
        self.blend_out_theta_offset = 0.0
        if self.turn_in_place:
            self.start_time_offset = self.rotation_time
        else:
            self.start_time_offset = 0.0

    def shift(self, duration):
        # print 'shifting:', duration, 'old start:', self.start_time, 'old end', self.end_time
        self.start_time += duration
        self.end_time += duration
        return duration

    def set_start_time(self, start_time):
        self.paused = False
        return self.shift(start_time - self.start_time)

    def pause(self):
        if self.paused:
            return
        self.paused = True
        self.pause_start_time = sim.SimTime

    def resume(self):
        if not self.paused:
            return
        shift_time = sim.SimTime - self.pause_start_time
        self.paused = False
        self.shift(shift_time)

    def get_time_to_target(self):
        return self.end_time - sim.SimTime if not self.paused else 1e8

    def interpolate(self, sim_time, blending=True):
        # type: (float) -> vcMatrix
        if self.paused:
            return self.pos_mtx

        rel_time = sim_time - self.start_time
        blend_in = False
        blend_out = False
        self.rotating = False

        # Added (KIVA)
        if self.turn_in_place and rel_time <= self.rotation_time + self.unload_time + self.load_time:
            # if self.turn_in_place and rel_time <= self.rotation_time:
            t = 0.0
            if rel_time > self.unload_time and rel_time <= self.rotation_time + self.unload_time:
                self.rotating = True
        elif blending and rel_time <= self.blend_in_end_time_offset:
            blend_in = True
            t = self.blend_in_end_time_offset - self.start_time_offset  # interpolate to the end of blend
        elif blending and self.blend_out_radius != 0 and rel_time >= self.blend_out_start_time_offset:
            blend_out = True
            # t = self.blend_out_start_time_offset - self.start_time_offset # interpolate to the start of blend
        else:
            t = rel_time - self.start_time_offset

        pos = self.pos_vec
        if blend_out:
            pos = self.blend_out_pos
        elif rel_time < self.transition_time:
            pos.X = self.init_pos_x + self.init_pos_x_offset + self.velocity_x * t
            pos.Y = self.init_pos_y + self.init_pos_y_offset + self.velocity_y * t
            pos.Z = self.init_pos_z + self.init_pos_z_offset + self.velocity_z * t
        else:
            pos.X = self.target_vec.X
            pos.Y = self.target_vec.Y
            pos.Z = self.target_vec.Z

        '''
    if rel_time < self.rotation_time:
      self.orient_vec.Z = self.init_rot + self.velocity_rot*(rel_time)
    else:
      self.orient_vec.Z = self.target_rotation     
    '''

        # Added (KIVA)
        # Unloading
        if rel_time <= self.unload_time and self.unload_time != 0.0:
            travel_controller.move_lift_interpolate("unload", rel_time)
            self.orient_vec.Z = self.init_rot
            # Rotating
        elif rel_time <= self.rotation_time + self.unload_time:

            self.orient_vec.Z = self.init_rot + self.velocity_rot * (rel_time - self.unload_time)
        # Loading
        elif rel_time <= self.rotation_time + self.unload_time + self.load_time and self.load_time != 0.0:

            travel_controller.move_lift_interpolate("load", rel_time - self.rotation_time - self.unload_time)
        else:
            self.orient_vec.Z = self.target_rotation

        self.pos_mtx.P = pos
        self.pos_mtx.WPR = self.orient_vec

        if blend_in:

            z = -self.blend_in_z_velocity * (self.blend_in_end_time_offset - rel_time)
            self.pos_mtx.translateRel(0, -self.blend_in_radius, z)
            self.pos_mtx.rotateRelZ(self.blend_in_ang_velocity * rel_time + self.blend_in_theta_offset)
            self.pos_mtx.translateRel(0, self.blend_in_radius, 0)
        elif blend_out:
            z = self.blend_out_z_velocity * (rel_time - self.blend_out_start_time_offset)
            self.pos_mtx.translateRel(0, -self.blend_out_radius, z)
            self.pos_mtx.rotateRelZ(self.blend_out_ang_velocity * (
                        rel_time - self.blend_out_start_time_offset) + self.blend_out_theta_offset)
            self.pos_mtx.translateRel(0, self.blend_out_radius, 0)
        return self.pos_mtx


class TrafficService(object):
    COLLISION_TYPE_NONE = 0
    COLLISION_TYPE_SLOW = 1
    COLLISION_TYPE_HEAD_TO_HEAD = 2
    COLLISION_TYPE_FOLLOW = 3
    COLLISION_TYPE_STOP = 4
    COLLISION_TYPE_OTHER_STOP = 5

    def __init__(self, tc_comp, update_signal):
        # type: (vcComponent, vcStringSignal) -> None
        self._comp = getComponent()
        enabled_p = tc_comp.getProperty('Avoidance::Enabled')
        self.enabled = enabled_p and enabled_p.Value
        self.traffic_signal = tc_comp.findBehaviour('TrafficUpdate')  # type: vcComponentSignal
        self.update_signal = update_signal
        self._next_col_time_p = validate_property(comp, VC_REAL, 'Nav::NextCollisionTime', visible=False, unit='Time')
        self._next_col_res_p = validate_property(comp, 'Ref<Component>', 'Nav::CollidingParty', visible=False)
        self.collision_distance_p = validate_property(comp, VC_REAL, 'Nav::CollisionDistance', visible=False,
                                                      unit='Distance')
        self.avoiding_p = validate_property(comp, 'Ref<Component>', 'Nav::Avoiding', visible=False)
        self.collision_type_p = validate_property(comp, VC_INTEGER, 'Nav::CollisionType', visible=False)
        self.collision_signal = comp.findBehaviour('Collision')  # type: vcComponentSignal
        self.next_col_time = 1e8  # 1e8 seconds = ~3 years
        self.other_resource = None  # type: vcComponent
        self.collision_type = TrafficService.COLLISION_TYPE_NONE

        self.avoiding_p.Value = None
        self._next_col_res_p.Value = None
        self._next_col_time_p.Value = -1.0

    def fetch_traffic_info(self, exclude_comp=None):
        # request traffic info
        if not self.enabled:
            return
        self._next_col_time_p.Value = -1.0
        self._next_col_res_p.Value = exclude_comp
        self.traffic_signal.signal(self._comp)
        # print comp.Name, 'FETHCING TRAFFIC INFO...', sim.SimTime
        triggerCondition(lambda: self.update_signal.Value == 'traffic')
        self.update_info()
        # print comp.Name, 'RECEIVED...', self.next_col_time, self.other_resource
        # delay(0.1) # to avoid constant check

    def update_available(self):
        return self.collision_signal.Value or self.update_signal.Value == 'traffic'

    def update_status(self):
        # use to notify others when stopping/pausing
        # print comp.Name, 'SENDING UPDATE STATUS!!!!!'
        self.traffic_signal.signal(self._comp)
        delay(0)

    def update_info(self):
        if not self.enabled:
            return
        if self.collision_signal.Value:
            # signal received from other resource
            self.other_resource = self.collision_signal.Value
            self.next_col_time = sim.SimTime  # now
            self.collision_distance_p.Value = self.other_resource.getProperty('Nav::CollisionDistance').Value
            col_type = self.other_resource.getProperty('Nav::CollisionType').Value
            if col_type == TrafficService.COLLISION_TYPE_OTHER_STOP:
                self.collision_type = TrafficService.COLLISION_TYPE_STOP
            else:
                self.collision_type = TrafficService.COLLISION_TYPE_SLOW
            self.collision_type_p.Value = self.collision_type
            self.collision_signal.Value = None
            # print comp.Name, 'Collision signal update:', 'time:', self.next_col_time, 'type:', self.collision_type, 'party:', self.other_resource
        elif self.update_signal.Value == 'traffic':
            # signal received from TC
            t = self._next_col_time_p.Value
            self.next_col_time = 1e8 if t < 0 else t
            self.collision_type = self.collision_type_p.Value
            self.other_resource = self._next_col_res_p.Value
            self.update_signal.Value = ''
            # print comp.Name, 'Traffic info updated:', 'time:', self.next_col_time, '/', sim.SimTime, 'type:', self.collision_type, 'party:', self.other_resource

    def occupy_position(self, pos, hdg):
        # type: (vcVector, vcVector) -> None
        if self.enabled:
            travel_controller.path_id_p.Value = 0
            travel_controller.destination_p.Value = pos
            travel_controller.hdg_p.Value = hdg
            # print comp.Name, 'OCCUPYING...', pos.X, pos.Y, pos.Z
            self.update_status()

    def release_position(self):
        if self.enabled:
            travel_controller.path_id_p.Value = -1
            self.update_status()

    def reset(self):
        self.next_col_time = -1.0


class TravelScheduler(object):
    EVENT_AREA_CROSSING = 1
    EVENT_COLLISION = 2
    EVENT_DESTINATION = 3

    def __init__(self, travel_controller, traffic_service, collision_manager):
        # type: (TravelController, TrafficService, CollisionManager) -> None
        self.ts = traffic_service
        self.tc = travel_controller
        self.cm = collision_manager
        self._sim = sim

    def reset(self):
        self.time_to_target = 0.0
        self.next_event_time = 0.0
        self.next_event = None
        self.in_collision = False
        self.area_group_entry = False
        self.path = self.tc.active_path  # type: Path
        self.collision_start_time = 0.0

    def update(self, fetch_traffic=True):
        # Added (Emergency Stop)
        if comp.EmergencyStop:
            return

        ttt = self.path.get_time_to_target(self._sim.SimTime)
        ttna = self.path.get_time_to_next_area2()
        if self.in_collision:
            if self.cm.state == CollisionManager.STATE_STOPPED and (
                    self._sim.SimTime + 0.001) - self.cm.last_state_time < self.cm.stop_timer:
                ttnc = self.cm.stop_timer - (self._sim.SimTime - self.cm.last_state_time)
            else:
                # select limiting sampling interval
                spd = self.tc.cur_spd_p.Value
                t1 = self.cm.sampling_interval
                t2 = self.cm.sampling_travel / spd if spd > 0 else t1
                ttnc = t2 if t2 < t1 else t1
        elif ttt > 0:
            if fetch_traffic:
                self.ts.fetch_traffic_info()
            else:
                self.ts.update_info()
            ttnc = self.ts.next_col_time - self._sim.SimTime
        else:
            ttnc = 1e8

        # print comp.Name, 'ttt', ttt, 'ttna', ttna, 'ttnc', ttnc, 'now:', sim.SimTime
        if ttna > ttt < ttnc:
            self.next_event = TravelScheduler.EVENT_DESTINATION
            self.next_event_time = ttt
        elif ttna <= ttnc:
            self.next_event = TravelScheduler.EVENT_AREA_CROSSING
            self.next_event_time = ttna
            self.area_group_entry = self.is_area_group_entry()
        else:
            self.next_event = TravelScheduler.EVENT_COLLISION
            self.next_event_time = ttnc
        self.time_to_target = ttt
        # print 'next_event', self.next_event

    def is_area_group_entry(self):
        if self.tc.current_area and self.path.next_area:
            return self.path.next_area.group and self.tc.current_area.group != self.path.next_area.group
        elif self.path.next_area:
            return bool(self.path.next_area.group)
        else:
            return False


class CollisionManager(object):
    STATE_DISABLED = 0
    STATE_FREE = 1
    STATE_SCANNING = 2
    STATE_CAUTIOUS = 3
    STATE_SYNCING = 4
    STATE_STOPPED = 5
    STATE_IGNORING = 6

    # STATE_WAITING = 7

    def __init__(self, connected_tc):
        # type: (vcComponent) -> None

        self.connected_tc = connected_tc
        self.speed_setting = -1.0  # speed setting, -1.0 = no restrictions
        enabled_p = connected_tc.getProperty('Avoidance::Enabled') if connected_tc else None
        self.state = self.STATE_FREE if enabled_p and enabled_p.Value else self.STATE_DISABLED
        # self._state_functions = [None, self.state_free]
        self.slow_detector = None
        self.stop_detector = None
        self.stop_timer = 0.0
        self.allow_ignoring = True
        self.sampling_interval = 0.0
        self.sampling_travel = 0.0

        self.avoiding_p = validate_property(comp, 'Ref<Component>', 'Nav::Avoiding', visible=False)
        self.scanning_p = validate_property(comp, VC_BOOLEAN, 'Nav::Scanning', visible=False)
        self.avoid_speed_p = comp.getProperty('MoveSpeedAvoidance')
        self.hdg_p = comp.getProperty('Nav::Heading')
        self.rotating_p = comp.getProperty('Nav::Rotating')
        self.collision_type_p = comp.getProperty('Nav::CollisionType')
        self.near_resources = []  # type: List[TrackedResource]
        self.last_state_time = 0.0
        self.prev_avoiding = None  # type: vcComponent
        self.tracked_resources = []
        self.reset()

    def initialize(self):
        if self.state == self.STATE_DISABLED:
            return
        st_p = self.connected_tc.getProperty('Avoidance::StopTimer')
        self.stop_timer = abs(st_p.Value)
        idl_p = self.connected_tc.getProperty('Avoidance::IgnoreDeadLocks')
        self.allow_ignoring = idl_p.Value
        msi_p = self.connected_tc.getProperty('Avoidance::MaxSamplingInterval')
        mti_p = self.connected_tc.getProperty('Avoidance::MaxTravelInterval')
        if msi_p.Value < 0.01:
            msi_p.Value = 0.01
        if mti_p.Value < 10.0:
            mti_p = 10.0
        self.sampling_interval = msi_p.Value
        self.sampling_travel = mti_p.Value

        slow_range_p = self.connected_tc.getProperty('Avoidance::SlowSensorRange')
        slow_side_p = self.connected_tc.getProperty('Avoidance::SlowSensorSideClearance')
        stop_range_p = self.connected_tc.getProperty('Avoidance::StopSensorRange')
        stop_side_p = self.connected_tc.getProperty('Avoidance::StopSensorSideClearance')
        detect_height = comp.BoundDiagonal.Z * 2
        self.slow_detector = DetectorVolume(slow_range_p.Value, slow_side_p.Value, detect_height, 'Slow')
        self.stop_detector = DetectorVolume(stop_range_p.Value, stop_side_p.Value, detect_height, 'Stop')
        self._load_resources()

    def _load_resources(self):
        if self.state != self.STATE_DISABLED:
            resources_p = self.connected_tc.getProperty('TrackedResources')
            if resources_p:
                self.tracked_resources = [TrackedResource(c) for c in resources_p.Value if c != comp]
            else:
                error(
                    'Connected Transport Controller has no required property "TrackedResources". Please update the controller.')

    def evaluate(self, wpm, current_area=None):
        # type: (vcMatrix, Optional[NavArea]) -> bool
        ''' wpm: position of the resource. X-axis pointing to travel direction. '''
        # Added (Pathway avoidance property)
        if current_area:
            current_area_component = current_area.component
            current_area_avoidance = current_area_component.getProperty("Avoidance")
            if current_area_avoidance:
                if (not current_area_avoidance.Value) or (
                        current_area_component.LimitCapacity and current_area_component.Capacity == 1):
                    return self._set_state(self.STATE_IGNORING)

        if self.state == self.STATE_DISABLED:
            return False

        if self.state == self.STATE_STOPPED:
            if sim.SimTime - self.last_state_time < self.stop_timer:
                # print comp.Name, '*** STOPPED TIMER ***'
                return False  # forced stop time

        map(lambda r: r.update_pos(wpm.P), self.tracked_resources)

        self.near_resources = [r for r in self.tracked_resources if
                               (r.WorldPositionMatrix.P - wpm.P).length() <= 1.1 * (
                                           self.slow_detector.range + r.bound_radius)]
        if not self.near_resources:
            # print comp.Name, '!!!! NO NEAR RESOURCES !!!!'
            return self._set_state(self.STATE_FREE)

        self.near_resources.sort(key=lambda r: (r.WorldPositionMatrix.P - wpm.P).length())
        next_state = self.STATE_SCANNING

        # behind = (other_res.WorldPositionMatrix.P - wpm.P)*self.hdg_p.Value < 0
        if self.state == self.STATE_IGNORING:
            stop_result_comp = self.stop_detector.scan(wpm, self.near_resources)
            if stop_result_comp:  # or self.body_detector.scan(wpm, near_resources):
                # print comp.Name, '!!!!!!!!!! IGNORE TEST', self.prev_avoiding, '=', stop_result_comp
                if self.prev_avoiding == stop_result_comp:
                    # print comp.Name, '*****IGNORING*****'
                    next_state = self.STATE_IGNORING
                    self.speed_setting = self.avoid_speed_p.Value
                else:
                    # print comp.Name, '========> NEW RESOURCE DETECTED!'
                    # scanned other resource
                    next_state = self.STATE_STOPPED
                    self.speed_setting = 0.0
                    self.avoiding_p.Value = stop_result_comp
            else:
                next_state = self.STATE_CAUTIOUS
                self.speed_setting = self.avoid_speed_p.Value
                # print comp.Name, '***** END IGNORING --> CAUTIOUS*****'
        elif self.slow_detector.scan(wpm, self.near_resources):
            if self.stop_detector.scan(wpm, self.near_resources):
                result = self.stop_detector.result
                next_state = self.STATE_STOPPED
                self.speed_setting = 0.0
                self.avoiding_p.Value = result.component
            else:
                next_state = self.STATE_CAUTIOUS
                self.speed_setting = self.avoid_speed_p.Value
                self.avoiding_p.Value = self.slow_detector.result.component

        if self.allow_ignoring and next_state == self.STATE_STOPPED and (
                self.state == self.STATE_STOPPED or self.stop_timer == 0):
            # print comp.Name, 'Evaluating IGNORE CONDITION'
            # Evaluate if can ignore or should wait
            other_res = self.stop_detector.result
            other_hdg = other_res.hdg_p.Value
            if self.stop_detector.result.is_ghost or (
                    other_res.is_moving() and other_res.collision_type_p.Value != self.STATE_IGNORING):
                pass
            elif other_res.avoiding_p.Value == comp:
                # if not other_res.getProperty('Nav::Rotating').Value or other_res.getProperty('Nav::Rotating').Value and self.rotating_p.Value:
                head_to_head = other_hdg * self.hdg_p.Value < 0
                # print comp.Name, 'H2H:', head_to_head, '/', other_hdg*self.hdg_p.Value
                if head_to_head:
                    if not other_res.rotating_p.Value:
                        next_state = self.STATE_IGNORING
                        self.speed_setting = self.avoid_speed_p.Value
                    elif self.rotating_p.Value and self._compare_angles(wpm.P, other_res):  # both rotating
                        next_state = self.STATE_IGNORING
                        self.speed_setting = self.avoid_speed_p.Value
                elif self._compare_angles(wpm.P, other_res):
                    # If side to side, heading same direction (crossing paths)
                    next_state = self.STATE_IGNORING
                    self.speed_setting = self.avoid_speed_p.Value
            elif self.is_circular_collision(comp, other_res.component):  # check for chained collision
                next_state = self.STATE_IGNORING
                self.speed_setting = self.avoid_speed_p.Value
            elif current_area and other_res.component in current_area.queue_p.Value:
                next_state = self.STATE_IGNORING
                self.speed_setting = self.avoid_speed_p.Value
        elif next_state == self.STATE_CAUTIOUS and self.state != self.STATE_IGNORING:
            # Set following: check if state must be set to self.STATE_SYNCING
            self.slow_detector.scan(wpm, self.near_resources, all_hits=True)
            v = 1.0e4
            avoid_spd = self.avoid_speed_p.Value
            for other_res in self.slow_detector.results:  # type: TrackedResource
                if self._follow(wpm.P, other_res) and not other_res.rotating_p.Value:
                    v_x = other_res.cur_speed_p.Value * other_res.hdg_p.Value * self.hdg_p.Value
                    if v_x <= 0:
                        v_x = avoid_spd
                    if 0 < v_x < v:
                        v = v_x
                    next_state = self.STATE_SYNCING
                elif avoid_spd < v:
                    v = avoid_spd
            if next_state == self.STATE_SYNCING:
                self.speed_setting = round(v)
                # print comp.Name, '----- FOLLOWING ----', 'Speed setting:', self.speed_setting
        # print comp.Name, 'EVALUATED. STATE', next_state, 'spd', self.speed_setting
        return self._set_state(next_state)

    def is_circular_collision(self, subject_res, res, previous=None):
        # type: (vcComponent, vcComponent) -> bool
        avoiding = res.getProperty('Nav::Avoiding').Value
        # print comp.Name, 'Chain Testing:', res.Name, '/', avoiding
        if avoiding and avoiding != previous and res.getProperty('Nav::Paused').Value:
            if avoiding == subject_res:
                return True
            return self.is_circular_collision(subject_res, avoiding, res)
        return False

    def time_to_next_potential_hit(self, wpm):
        moving_resources = [r for r in self.tracked_resources if not r.paused_p.Value]
        moving_resources.sort(key=lambda r: (r.WorldPositionMatrix.P - wpm.P).length())
        if not moving_resources:
            return 1e8
        nearest = moving_resources[0]
        d = (nearest.WorldPositionMatrix.P - wpm.P).length() - (self.slow_detector.range + nearest.bound_radius)
        t = d / nearest.move_speed_p.Value
        return 0.4 if t < 0.4 else t

    def reset(self):
        self.avoiding_p.Value = None
        self.scanning_p.Value = False
        self.prev_avoiding = None
        self.speed_setting = -1.0
        if self.state != self.STATE_DISABLED:
            self.state = self.STATE_FREE
        self.last_state_time = 0.0

    def _follow(self, this_pos, other_res):
        # type: (vcVector, TrackedResource) -> bool
        ''' Return true if both heading to same direction and other is more ahead than this resource '''
        hdg = self.hdg_p.Value
        other_hdg = other_res.hdg_p.Value
        follow = False
        if other_hdg * hdg > 0:
            diff_vec = other_res.WorldPositionMatrix.P - this_pos
            other_ahead = diff_vec * self.hdg_p.Value
            follow = other_ahead > 0 and (-1 * diff_vec) * other_res.hdg_p.Value <= other_ahead
        return follow

    def _compare_angles(self, this_pos, other_res):
        # type: (vcVector, TrackedResource) -> bool
        ''' Return True if other resource has lower angle to this resource'''
        diff_vec = other_res.WorldPositionMatrix.P - this_pos
        angle1 = self.hdg_p.Value.angle(diff_vec)
        angle2 = other_res.hdg_p.Value.angle(-1 * diff_vec)
        return angle2 <= angle1

    def _set_state(self, new_state):
        # type: (...) -> bool
        ''' Returns true if state changed '''
        # print comp.Name, 'STATE', self.state, new_state
        state_change = self.state != new_state
        if state_change:
            self.last_state_time = sim.SimTime
            if new_state <= self.STATE_SCANNING:
                self.avoiding_p.Value = None
                self.speed_setting = -1.0
            self.scanning_p.Value = False if new_state < self.STATE_SCANNING else True
            # self._state_functions[new_state]
            self.state = new_state
            self.collision_type_p.Value = new_state
        self.prev_avoiding = self.avoiding_p.Value
        return state_change


class DetectorVolume(object):
    def __init__(self, detect_range=0.0, hori_clearance=0.0, vert_clearance=0.0, name=''):
        self.range = detect_range
        cbd = comp.BoundDiagonal
        self.bounds = vcVector.new(detect_range / 2.0, cbd.Y + hori_clearance, (cbd.Z + vert_clearance) / 2.0)
        self.name = name
        self.result = None  # type: TrackedResource # last scan result
        self.results = []  # type: List[TrackedResource]

    def scan(self, pos_mtx, resources, all_hits=False):
        # type: (vcMatrix, List[TrackedResource]) -> Optional[vcComponent]
        ''' pos_mtx: position of the resource. X-axis pointing to travel direction. Returns the first hit '''
        # hdg_mtx = travel_controller.get_hdg_matrix()
        # hdg_mtx.P = travel_controller.position_matrix.P

        # components.sort(key=lambda c: (c.WorldPositionMatrix.P - pos_mtx.P).length()) # nearest first

        if all_hits and self.results:
            self.results = []
        self.result = None

        test_wpm = vcMatrix.new(pos_mtx)
        test_wpm.translateRel(self.bounds.X, 0.0, self.bounds.Z)
        for other_resource in resources:
            n2_wpm = vcMatrix.new(other_resource.WorldPositionMatrix)
            bc = other_resource.BoundCenter
            n2_wpm.translateRel(bc.X, bc.Y, bc.Z)
            n2_bd = other_resource.BoundDiagonal
            if self._detect(test_wpm, self.bounds, n2_wpm, n2_bd):
                # print comp.Name, self.name, 'SCAN RESULT:', other_resource.component.Name
                self.result = other_resource
                if not all_hits:
                    return other_resource.component
                self.results.append(other_resource)

    @staticmethod
    def _detect(n1_wpm, n1_bd, n2_wpm, n2_bd):
        # type: (vcMatrix, vcVector, vcMatrix, vcVector) -> bool
        ''' Takes the center pos matrix (in WCS) and diagonal vector of the bounds/volumes as arguments'''
        hit = False
        n1n2 = n2_wpm.P - n1_wpm.P
        if abs(n1n2.Z) <= n1_bd.Z + n2_bd.Z:
            x_dot = n1_wpm.N * n2_wpm.N
            y_dot = n1_wpm.O * n2_wpm.O
            x1 = n1_bd.X + abs(x_dot * n2_bd.X) + abs(n1_wpm.N * n2_wpm.O * n2_bd.Y)
            y1 = n1_bd.Y + abs(y_dot * n2_bd.Y) + abs(n1_wpm.O * n2_wpm.N * n2_bd.X)
            if abs(n1_wpm.N * n1n2) <= x1 and abs(n1_wpm.O * n1n2) <= y1:
                x2 = n2_bd.X + abs(x_dot * n1_bd.X) + abs(n2_wpm.N * n1_wpm.O * n1_bd.Y)
                y2 = n2_bd.Y + abs(y_dot * n1_bd.Y) + abs(n2_wpm.O * n1_wpm.N * n1_bd.X)
                if abs(n2_wpm.N * n1n2) <= x2 and abs(n2_wpm.O * n1n2) <= y2:
                    hit = True
        return hit


class TrackedResource(object):
    COMP = getComponent()

    def __init__(self, component):
        # type: (vcComponent) -> None
        self.component = component
        self.hdg_p = component.getProperty('Nav::Heading')
        self.paused_p = component.getProperty('Nav::Paused')
        self.rotating_p = component.getProperty('Nav::Rotating')
        self.move_speed_p = component.getProperty('MoveSpeed')
        self.cur_speed_p = component.getProperty('CurrentSpeed')
        self.avoiding_p = component.getProperty('Nav::Avoiding')
        self.scanning_p = component.getProperty('Nav::Scanning')
        self.collision_signal = component.getBehaviour('Collision')
        self.path_id_p = component.getProperty('Nav::PathId')
        self.occupy_loc_p = component.getProperty('Nav::DestinationRequest')
        self.collision_type_p = component.getProperty('Nav::CollisionType')
        self.is_ghost = False

        # Read detect bound (within range) data from resource's TC
        tc = component.findBehaviour('PMResource').ConnectedComponent  # type: vcComponent
        self.bound_radius = tc.getProperty('DetectBound').Value.length()  # a max value from comp origin

        # use actual bound data for sensoring
        self.WorldPositionMatrix = component.WorldPositionMatrix
        self.BoundCenter = component.BoundCenter
        self.BoundDiagonal = component.BoundDiagonal
        self.physical_pos = None  # type: vcVector

    def update_pos(self, comp_to_pos):
        # type: (vcVector) -> None
        self.component.update()
        # self.BoundCenter = self.component.BoundCenter
        # self.BoundDiagonal = self.component.BoundDiagonal
        wpm = self.component.WorldPositionMatrix
        self.is_ghost = False
        if self.path_id_p.Value == 0:
            occupy_pos = self.occupy_loc_p.Value
            d1 = (comp_to_pos - occupy_pos).length()
            d2 = (comp_to_pos - wpm.P).length()
            if d1 < d2:
                self.physical_pos = wpm.P
                wpm.P = occupy_pos
                self.is_ghost = True
        self.WorldPositionMatrix = wpm

    def is_moving(self):
        # type: () -> bool
        return self.cur_speed_p.Value > 0 or self.rotating_p.Value


class GlobalAreaComponent(object):
    def __init__(self, detect_obstacles):
        # type: (bool) -> None
        self.Name = 'GLOBAL_AREA'
        self.DetectObstacles = VirtualProperty('DetectObstacles', detect_obstacles)
        self.OneWay = VirtualProperty('OneWay', False)
        self.LimitSpeed = VirtualProperty('LimitSpeed', False)
        self.SpeedLimit = VirtualProperty('SpeedLimit', 10000.0)
        self.LimitCapacity = VirtualProperty('LimitCapacity', False)
        self.Capacity = VirtualProperty('Capacity', 1000)
        self.UsedCapacity = VirtualProperty('UsedCapacity', 0)
        self.Queue = VirtualProperty('Queue', [])

    def getProperty(self, name):
        return getattr(self, name, None)

    def findBehaviour(self, name):
        return None


class VirtualProperty(object):
    def __init__(self, name, value):
        self.Name = name
        self.Value = value


ActionManager.execute_pick_action = execute_pick_action
ActionManager.execute_place_action = execute_place_action
ActionManager.execute_work_action = execute_work_action
ActionManager.execute_idle = execute_idle

ActionManager.execute_charge_step = execute_charge_step
ActionManager.execute_collect_step = execute_collect_step
ActionManager.execute_collect_tool_step = execute_collect_tool_step
ActionManager.execute_deliver_step = execute_deliver_step
ActionManager.execute_deliver_tool_step = execute_deliver_tool_step
ActionManager.execute_move_step = execute_move_step
ActionManager.execute_move_joint_step = execute_move_joint_step
ActionManager.execute_run_robot_routine_step = execute_run_robot_routine_step

ActionManager.execute_send_signal_step = execute_send_signal_step
ActionManager.execute_wait_step = execute_wait_step
ActionManager.execute_wait_signal_step = execute_wait_signal_step
ActionManager.execute_work_step = execute_work_step

action_manager = ActionManager(comp.findBehaviour('PMResource'))

available_p = comp.getProperty('Available')
available_p.OnChanged = on_availability_changed

power_enabled_p = comp.getProperty('Power::Enabled')
if power_enabled_p:
    power_enabled_p.OnChanged = toggle_power_prop_visibility
    charge_priority_p = comp.getProperty('Power::ChargerPriority')
    if charge_priority_p:
        charge_priority_p.OnChanged = on_charger_priority_changed

stats_dump_req_p = comp.getProperty('Nav::StatsDumpRequest')
stats_dump_req_p.OnChanged = stats_dump_request


# debugging tools for output messages and extra component properties
def on_debug_mode_changed(arg):
    for p in comp.Properties:
        if p.Name[0:9] == 'Mission::':
            p.IsVisible = arg.Value
            p.WritableWhenConnected = False
            p.WritableWhenDisconnected = False


debug_p = comp.getProperty('DEBUG')
if not debug_p:
    debug_p = comp.createProperty(VC_BOOLEAN, 'DEBUG')
debug_p.IsVisible = False
debug_p.OnChanged = on_debug_mode_changed
bindDebugProperty('DEBUG')


# Added (KIVA)
def OnSetRaycast(arg):
    raycast.MaxRange = transport_raycast_max_range.Value
    raycast.DetectionThreshold = transport_raycast_threshold.Value
    raycast.ShowRay = transport_raycast_show_raycast.Value


raycast = comp.findBehaviour("RaycastSensor")
transport_raycast_max_range = comp.getProperty("Transport::RaycastMaxRange")
transport_raycast_threshold = comp.getProperty("Transport::RaycastThreshold")
transport_raycast_show_raycast = comp.getProperty("Transport::ShowRaycast")

if transport_raycast_show_raycast:
    transport_raycast_max_range.OnChanged = OnSetRaycast
    transport_raycast_threshold.OnChanged = OnSetRaycast
    transport_raycast_show_raycast.OnChanged = OnSetRaycast


def OnChangeLockProductOrientation(arg):
    if transport_lock_product_orientation.Value != "None" and config:
        diff = comp.WorldPositionMatrix.WPR.Z - transport_product_orientation.Value
        payload = config.cont.HeadComponent
        if payload:
            matrix = payload.PositionMatrix
            matrix.rotateRelZ(diff)
            payload.PositionMatrix = matrix

    if transport_lock_product_orientation.Value != "Ground":
        comp.TurnInPlaceAngleThreshold = 180.0
    else:
        comp.TurnInPlaceAngleThreshold = 0.0


transport_product_orientation = comp.getProperty("Transport::ProductOrientation")
transport_lock_product_orientation = comp.getProperty("Transport::LockProductOrientation")
if transport_lock_product_orientation:
    transport_lock_product_orientation.OnChanged = OnChangeLockProductOrientation


# Added (Emergency Stop)
def emergency_stop_gui(writable):
    emergency_stop.WritableWhenConnected = writable
    emergency_stop.WritableWhenDisconnected = writable
    emergency_stop.WritableWhenSimulating = writable


def WhenEmergencyStop(arg):
    global manual_pause
    manual_pause = True

    global halt_state
    if comp.EmergencyStop and stats_manager:
        halt_state = stats_manager.statistics.State


emergency_stop = comp.getProperty("EmergencyStop")
emergency_stop.OnChanged = WhenEmergencyStop
looks = comp.getProperty("Looks")


def pick_place_tolerance_on_changed(show):
    tick = comp.getProperty("Transport::MoveTolerance")
    if show.Value:
        tick.IsVisible = True
    else:
        tick.IsVisible = False


if enable_pick_place_tolerance_prop:
    enable_pick_place_tolerance_prop.OnChanged = pick_place_tolerance_on_changed
