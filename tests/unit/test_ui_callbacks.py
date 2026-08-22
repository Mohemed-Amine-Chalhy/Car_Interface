from __future__ import annotations

from unittest.mock import Mock, patch

import pytest

from car_interface.adapters.base import ScanPoint
from car_interface.services.events import EventBroker, EventType, ServiceEvent
from car_interface.services.lidar_analysis import LidarAssessment
from car_interface.services.models import ServiceSnapshot
from car_interface.ui import app as ui


class FakeWidget:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs

    def pack(self, *args, **kwargs) -> None:
        return

    def grid(self, *args, **kwargs) -> None:
        return

    def columnconfigure(self, *args, **kwargs) -> None:
        return

    def rowconfigure(self, *args, **kwargs) -> None:
        return

    def add(self, *args, **kwargs) -> None:
        return

    def bind(self, *args, **kwargs) -> None:
        return

    def configure(self, *args, **kwargs) -> None:
        return

    def yview(self, *args, **kwargs) -> None:
        return

    def set(self, *args, **kwargs) -> None:
        return

    def tag_configure(self, *args, **kwargs) -> None:
        return


class FakeVariable:
    def __init__(self, value=None) -> None:
        self.value = value

    def get(self):
        return self.value

    def set(self, value) -> None:
        self.value = value


def _window() -> ui.CarInterfaceWindow:
    window = object.__new__(ui.CarInterfaceWindow)
    window.root = Mock()
    window.service = Mock(events=EventBroker())
    window.simulation = True
    window.max_speed_percent = 50
    window._closing = False
    window._shutdown_finished = False
    window._last_scan = ()
    for name in (
        "phase_label",
        "connection_label",
        "vehicle_status_label",
        "lidar_status_label",
        "controller_status_label",
        "fault_label",
        "obstacle_label",
        "speed_label",
        "connect_button",
        "disconnect_button",
        "arm_button",
        "disarm_button",
        "reset_estop_button",
        "speed_scale",
        "steering_scale",
        "steering_label",
        "live_speed_label",
        "live_steering_label",
        "direction_status_label",
        "brake_status_label",
        "log_text",
    ):
        setattr(window, name, Mock())
    window.speed_value = Mock()
    window.steering_value = Mock()
    window.direction_value = Mock(get=Mock(return_value="reverse"))
    window.auto_stop_value = Mock()
    window.lidar_canvas = Mock()
    window.log_text.index.return_value = "5.0"
    return window


@pytest.mark.parametrize("simulation", [True, False])
def test_widget_tree_builds_headlessly_for_both_runtime_modes(simulation: bool) -> None:
    window = _window()
    window.simulation = simulation
    constructors = dict.fromkeys(
        (
            "Frame",
            "Label",
            "Notebook",
            "LabelFrame",
            "Button",
            "Scale",
            "Radiobutton",
            "Checkbutton",
            "Scrollbar",
        ),
        FakeWidget,
    )
    with (
        patch.multiple(ui.ttk, **constructors),
        patch.object(ui.tk, "IntVar", FakeVariable),
        patch.object(ui.tk, "StringVar", FakeVariable),
        patch.object(ui.tk, "BooleanVar", FakeVariable),
        patch.object(ui.tk, "DoubleVar", FakeVariable),
        patch.object(ui.tk, "Canvas", FakeWidget),
        patch.object(ui.tk, "Text", FakeWidget),
    ):
        window._build_widgets()

    assert isinstance(window.notebook, FakeWidget)
    assert isinstance(window.speed_scale, FakeWidget)
    assert isinstance(window.lidar_canvas, FakeWidget)
    assert isinstance(window.vehicle_status_label, FakeWidget)
    assert isinstance(window.live_speed_label, FakeWidget)
    assert hasattr(window, "simulated_distance") is simulation


def test_style_configuration_tolerates_unavailable_theme() -> None:
    window = _window()
    style = Mock()
    style.theme_use.side_effect = ui.tk.TclError
    with patch.object(ui.ttk, "Style", return_value=style):
        window._configure_styles()
    configured_styles = {call.args[0] for call in style.configure.call_args_list}
    assert {
        "App.TFrame",
        "Card.TFrame",
        "Title.TLabel",
        "Metric.TLabel",
        "Primary.TButton",
        "Emergency.TButton",
        "TNotebook.Tab",
    } <= configured_styles
    mapped_styles = {call.args[0] for call in style.map.call_args_list}
    assert {"Primary.TButton", "Emergency.TButton", "TNotebook.Tab"} <= mapped_styles


def test_window_configuration_and_control_callbacks_are_view_only() -> None:
    window = _window()
    window._configure_window()
    window.root.title.assert_called_once_with("Car Interface — SIMULATION")
    window.root.geometry.assert_called_once_with("1120x760")
    window.root.minsize.assert_called_once_with(900, 640)

    window._on_speed("12.6")
    window.speed_label.config.assert_called_with(text="13%")
    window.service.set_speed.assert_called_with(13)
    window._on_steering("-12.6")
    window.steering_label.config.assert_called_with(text="-13%")
    window.service.set_steering.assert_called_with(-13)
    window._on_direction()
    window.service.set_direction.assert_called_once_with("reverse")


def test_estop_reset_requires_confirmation() -> None:
    window = _window()
    with patch.object(ui.messagebox, "askyesno", return_value=False):
        window._confirm_estop_reset()
    window.service.reset_emergency_stop.assert_not_called()

    with patch.object(ui.messagebox, "askyesno", return_value=True):
        window._confirm_estop_reset()
    window.service.reset_emergency_stop.assert_called_once_with()


def test_refresh_routes_events_snapshot_and_reschedules() -> None:
    window = _window()
    assessment = LidarAssessment(
        points=(ScanPoint(0, 100),),
        closest_in_path_cm=100,
        closest_angle_degrees=0,
        stale=False,
    )
    window.service.events.publish(EventType.SCAN, assessment)
    window.service.events.publish(EventType.LOG, "ready")
    window.service.snapshot.return_value = ServiceSnapshot()
    window._draw_lidar = Mock()
    window._append_log = Mock()
    window._apply_snapshot = Mock()

    window._refresh()

    assert window._last_scan == assessment.points
    window._draw_lidar.assert_called_once_with()
    window._append_log.assert_called_once_with("ready", EventType.LOG)
    window._apply_snapshot.assert_called_once_with(window.service.snapshot.return_value)
    window.root.after.assert_called_once_with(window.REFRESH_MS, window._refresh)

    window._closing = True
    window.root.after.reset_mock()
    window._refresh()
    window.root.after.assert_not_called()


def test_handle_event_ignores_unrecognized_payloads_and_state_events() -> None:
    window = _window()
    window._draw_lidar = Mock()
    window._append_log = Mock()
    window._handle_event(ServiceEvent(EventType.SCAN, object(), 1.0))
    window._handle_event(ServiceEvent(EventType.STATE, "state", 1.0))
    window._draw_lidar.assert_not_called()
    window._append_log.assert_not_called()


def test_handle_event_hides_routine_heartbeats_but_keeps_operator_events() -> None:
    window = _window()
    window._append_log = Mock()

    window._handle_event(ServiceEvent(EventType.COMMAND, "ACK seq=42 HBT", 1.0))
    window._handle_event(ServiceEvent(EventType.COMMAND, "ACK seq=43 EST", 1.0))

    window._append_log.assert_called_once_with("ACK seq=43 EST", EventType.COMMAND)


@pytest.mark.parametrize(
    "snapshot",
    [
        ServiceSnapshot(
            phase="safe_connected",
            vehicle_connected=True,
            lidar_connected=True,
            controller_connected=True,
            brake_active=True,
        ),
        ServiceSnapshot(
            phase="driving",
            vehicle_connected=True,
            lidar_connected=True,
            controller_connected=True,
            brake_active=False,
            closest_obstacle_cm=42.25,
        ),
        ServiceSnapshot(
            phase="fault",
            vehicle_connected=True,
            fault="sensor lost",
            estop_active=True,
        ),
    ],
)
def test_snapshot_projection_updates_all_operator_controls(snapshot: ServiceSnapshot) -> None:
    window = _window()
    window._apply_snapshot(snapshot)

    window.phase_label.config.assert_any_call(text=f"State: {snapshot.phase.upper()}")
    window.connection_label.config.assert_called_once()
    window.fault_label.config.assert_called_once_with(text=snapshot.fault or "")
    if snapshot.fault:
        window.fault_label.grid.assert_called_once_with()
        window.fault_label.grid_remove.assert_not_called()
    else:
        window.fault_label.grid_remove.assert_called_once_with()
        window.fault_label.grid.assert_not_called()
    window.auto_stop_value.set.assert_called_once_with(snapshot.auto_stop_enabled)
    window.live_speed_label.config.assert_called_once_with(text=f"{abs(snapshot.speed_percent)}%")
    expected_steering = (
        "0%" if snapshot.steering_percent == 0 else f"{snapshot.steering_percent:+d}%"
    )
    window.live_steering_label.config.assert_called_once_with(text=expected_steering)
    window.direction_status_label.config.assert_called_once_with(text=snapshot.direction.upper())
    window.brake_status_label.config.assert_called_once_with(
        text="ENGAGED" if snapshot.brake_active else "RELEASED",
        style="Warning.TLabel" if snapshot.brake_active else "Safe.TLabel",
    )
    for label, connected, name in (
        (window.vehicle_status_label, snapshot.vehicle_connected, "Vehicle"),
        (window.lidar_status_label, snapshot.lidar_connected, "Lidar"),
        (window.controller_status_label, snapshot.controller_connected, "Controller"),
    ):
        label.config.assert_called_once_with(
            text=f"{'●' if connected else '○'} {name}",
            style="DeviceOn.TLabel" if connected else "DeviceOff.TLabel",
        )
    window.speed_scale.config.assert_called_once()
    window.steering_scale.config.assert_called_once()
    if snapshot.brake_active or snapshot.estop_active:
        window.speed_value.set.assert_called_once_with(0)
    if snapshot.closest_obstacle_cm is None:
        window.obstacle_label.config.assert_called_once_with(text="Closest path obstacle: —")
    else:
        window.obstacle_label.config.assert_called_once_with(
            text=f"Closest path obstacle: {snapshot.closest_obstacle_cm:.1f} cm"
        )


def test_lidar_rendering_filters_range_and_colors_distance_bands() -> None:
    window = _window()
    canvas = window.lidar_canvas
    canvas.winfo_width.return_value = 500
    canvas.winfo_height.return_value = 300
    window._last_scan = (
        ScanPoint(0, 20),
        ScanPoint(-20, 100),
        ScanPoint(30, 200),
        ScanPoint(0, 401),
    )

    window._draw_lidar()

    canvas.delete.assert_called_once_with("all")
    assert canvas.create_arc.call_count == 4
    assert canvas.create_polygon.call_count == 2
    canvas.create_rectangle.assert_called_once()
    scan_points = [
        call
        for call in canvas.create_oval.call_args_list
        if call.kwargs.get("tags") == ("scan-point",)
    ]
    assert len(scan_points) == 3
    assert [call.kwargs["fill"] for call in scan_points] == [
        "#ff5252",
        "#ffd740",
        "#69f0ae",
    ]
    assert any(
        call.kwargs.get("tags") == ("projected-path",)
        for call in canvas.create_polygon.call_args_list
    )


def test_log_helpers_toggle_text_widget_state() -> None:
    window = _window()
    window._append_log("connected", EventType.CONNECTION)
    window.log_text.insert.assert_called_once()
    assert "CONNECTION: connected" in window.log_text.insert.call_args.args[1]
    assert window.log_text.insert.call_args.args[2] == EventType.CONNECTION.value
    window.log_text.see.assert_called_once_with(ui.tk.END)

    window._clear_log()
    window.log_text.delete.assert_called_once_with("1.0", ui.tk.END)


def test_close_flow_is_confirmed_disables_children_and_starts_shutdown() -> None:
    window = _window()
    good_child = Mock()
    bad_child = Mock()
    bad_child.configure.side_effect = AttributeError
    window.winfo_children = Mock(return_value=[good_child, bad_child])
    thread = Mock()
    with (
        patch.object(ui.messagebox, "askokcancel", return_value=True),
        patch.object(ui.threading, "Thread", return_value=thread) as thread_type,
    ):
        window._request_close()

    assert window._closing
    good_child.configure.assert_called_once_with(state=ui.tk.DISABLED)
    thread_type.assert_called_once_with(
        target=window._shutdown_worker,
        name="ui-shutdown",
        daemon=True,
    )
    thread.start.assert_called_once_with()
    window.root.after.assert_called_once_with(50, window._wait_for_shutdown)


def test_close_cancellation_shutdown_worker_and_completion_polling() -> None:
    window = _window()
    with patch.object(ui.messagebox, "askokcancel", return_value=False):
        window._request_close()
    assert not window._closing

    window._closing = True
    with patch.object(ui.messagebox, "askokcancel") as confirm:
        window._request_close()
    confirm.assert_not_called()

    window._shutdown_worker()
    assert window._shutdown_finished
    window._wait_for_shutdown()
    window.root.destroy.assert_called_once_with()

    window._shutdown_finished = False
    window._wait_for_shutdown()
    window.root.after.assert_called_once_with(50, window._wait_for_shutdown)
