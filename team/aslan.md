# Aslan: agent core + integration
You own: `agent.py`, `page.html`, `servo_bus.py`, `config.json`.

## Tasks, in order
1. [ ] **YOUR laptop runs both agents** in two terminals; the overhead camera and the follower plug into it. Push `regions.json` and `motions/` as backups
2. [ ] `python test_scenarios.py` → ALL PASS on your laptop
3. [ ] On your laptop: fake robot + real hand (see README "Run for real") (real network, fake arms). Order from each page, try Cut comms. ✅ **2:15**
4. [ ] Put Tanish's distances into `config.json`; tune `human_max_weight`, `weight_cost_k`
5. [ ] Switch the robot to `--arm so101` with Kazy and the human to `--arm hand --cam N`, both on your laptop. Full run ✅ **3:30**
6. [ ] Tier 2 with Tanish: `--cam N --roi ...` on the robot agent
7. [ ] Merge PRs and pushes; keep `test_scenarios.py` passing

## Known limits worth knowing
- Each object is in one slot; an order can't reuse a slot until **Reset run** (= restock)
- Comms-cut safety relies on the split itself (the human isn't assigned robot slots) plus the zone pause while connected
- Firewalls: if the pages work but agents never "find" each other, allow Python through the Windows/macOS firewall (UDP 9100/9101)
