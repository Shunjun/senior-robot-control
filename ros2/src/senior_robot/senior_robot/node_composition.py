"""Hardware-profile selection at the vehicle composition boundary."""


def motion_fusion_main() -> None:
    from senior_motion_control.motion_profile import select_motion_profile
    from senior_motion_estimation.motion_fusion import main

    main(select_motion_profile)
