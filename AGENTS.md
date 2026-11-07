# Instructions for AI coding assistants (Claude, Copilot, Cursor, ChatGPT...)

You are helping ONE member of a 5-person hackathon team. Deadline: **4:45pm code freeze, 6pm judging**.

1. **Read `README.md`, then `team/<your human's name>.md`.** That file is your task list.
2. **Only edit files your human owns** (see the owner table in README.md and the `OWNER:` line at the top of each file). If a change is needed in someone else's file, write the exact change in your reply so your human can post it in Discord. Don't edit the file yourself.
3. **Never change an interface** (the class/function signatures documented at the top of each file) without your human confirming with the file's owner.
4. **Keep it simple and working.** No new frameworks, no cloud services, no ML training. Stdlib + `feetech-servo-sdk` + `pyserial` (+ `opencv-python` in `vision.py` only).
5. **No central controller, ever.** No brokers, no shared server, no one laptop driving both arms. That's the judging criterion.
6. **Before pushing:** run `python test_scenarios.py` (no hardware needed). It must print `ALL PASS`.
7. **Git:** pull before you start, commit small, push often. Commit messages start with your human's name, e.g. `kazy: tune replay speed`. If you get a merge conflict in a file you don't own, take the owner's version.
8. Hardware facts: SO-101 arms, Feetech STS3215 servos, IDs 1-6 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper), 1 Mbaud, positions in raw ticks (4096 = 360°). The leader has a trigger handle instead of a gripper and **no motors are driven on it**.
