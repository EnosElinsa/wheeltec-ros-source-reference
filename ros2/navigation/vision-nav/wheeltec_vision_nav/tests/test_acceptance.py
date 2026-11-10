import pytest

from wheeltec_vision_nav.acceptance import summarize_trials, trial_template


def test_unrun_trials_never_become_success_or_measured_clearance():
    result = summarize_trials(trial_template())
    assert result["completed_trials"] == 0 and result["missing_trials"] == 27
    assert result["navigation_success_fraction"] is None
    assert result["minimum_clearance_m"] is None
    assert not result["protocol_complete"] and result["hardware_validation"] == "not_run"


def test_partial_runs_report_failure_reason_and_preserve_missing_measurements():
    trials = trial_template()
    trials["trials"][3].update(result="pass", run_directory="/runs/straight-1", elapsed_s=20,
                              minimum_clearance_m=.12, interventions=0)
    trials["trials"][4].update(result="fail", run_directory="/runs/straight-2", elapsed_s=25,
                              failure_reason="weak localization", interventions=1)
    result = summarize_trials(trials)
    assert result["navigation_success_fraction"] == .5
    assert result["failure_reasons"] == ["weak localization"]
    assert result["interventions_total"] == 1
    assert result["missing_trials"] == 25 and not result["protocol_complete"]
    assert result["stopping_distance_m"]["count"] == 0


def test_invalid_measurement_or_unbound_success_is_rejected():
    trials = trial_template()
    trials["trials"][0].update(result="pass")
    with pytest.raises(ValueError, match="evidence"):
        summarize_trials(trials)
    trials = trial_template()
    trials["trials"][0]["minimum_clearance_m"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        summarize_trials(trials)
