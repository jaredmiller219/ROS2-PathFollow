import pytest

import main


def test_ask_for_goal_distance_retries_invalid_then_returns_value(monkeypatch, capsys):
    answers = iter(("not a number", "-1", "nan", "0.75"))
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))

    assert main.ask_for_goal_distance() == pytest.approx(0.75)
    output = capsys.readouterr().out
    assert "Enter one valid number." in output
    assert "Distance must be finite and non-negative." in output


def test_ask_for_goal_distance_accepts_zero(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt: "0")

    assert main.ask_for_goal_distance() == 0.0


def test_main_starts_controller_with_requested_distance_and_args(monkeypatch):
    calls = []
    monkeypatch.setattr(main, "ask_for_goal_distance", lambda: 1.25)
    monkeypatch.setattr(
        main.RobotController,
        "run",
        lambda distance, args=None: calls.append((distance, args)),
    )

    main.main(args=["--ros-args"])

    assert calls == [(1.25, ["--ros-args"])]
