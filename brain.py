"""
brain.py - OWNER: Aslan. The robot agent's LLM "brain" (Claude), layered on top of its reflexes.

The shared allocation rule decides WHICH parcels are the robot's (that keeps both agents in agreement).
The brain decides WHAT TO DO NEXT with them: which one first, whether to wait for the human to clear
a nearby slot, and what to tell the human. It can only choose from options the rule already allows,
so it can never break the safety envelope (weight limit, hand-in-slot pause).

If Claude is slow (> timeout) or unreachable, the agent instantly falls back to its reflex rule:
the LLM link is just another comms link, and the robot keeps working when it drops.

    PowerShell:  $env:ANTHROPIC_API_KEY = "sk-ant-..."     (never commit the key)
    python agent.py --id robot ... --llm
"""
import json
import os
import time
import urllib.error
import urllib.request

API = "https://api.anthropic.com/v1/messages"
DEFAULT_MODEL = "claude-haiku-5-5"

SYSTEM = """You are the decision-making brain of an autonomous robot arm working in a small fulfilment cell
alongside one human co-worker. Paper-ball parcels sit in numbered slots 1-5 (slots are in a row, so
slot 2 is next to slots 1 and 3). Each parcel has a weight 1-10. A shared rule has already decided which
parcels are yours; you decide what to do next with them.

Goals, in order:
1. Safety: never work in or right next to the slot where the human's hand is (hand_zone). If your best
   option is in or adjacent to the human's hand_zone, pick a different option or wait briefly.
2. Throughput: heavier parcels first, and keep the cell moving. The human is slower when their learned
   pick time is high; don't make them wait on you.
3. Communication: if something notable happened (you missed a grab, the human took your parcel, the
   human went away or came back, you're waiting for them), tell the human in a few friendly words.

Always call choose_action. Keep reasoning to 1-2 short sentences that mention the facts you used."""

TOOL = {
    "name": "choose_action",
    "description": "Choose the robot's next action.",
    "input_schema": {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": ["claim", "wait"],
                       "description": "claim = start picking one of my_options now; wait = hold a few seconds"},
            "pick": {"type": "string", "description": "the pick id from my_options (required when action is claim)"},
            "reasoning": {"type": "string", "description": "1-2 short sentences: why, citing the facts used"},
            "say_to_human": {"type": "string", "description": "optional message for the human (max 12 words), or empty"},
        },
        "required": ["action", "reasoning"],
    },
}


class Brain:
    def __init__(self, model=None, key=None, timeout=4.0, mock=False):
        self.model = model or DEFAULT_MODEL
        self.key = key or os.environ.get("ANTHROPIC_API_KEY", "")
        self.timeout = timeout
        self.mock = mock
        self.calls = 0

    def decide(self, context):
        """Returns {"action", "pick", "reasoning", "say", "source", "ms"} or {"action": "fallback", "error"}."""
        t0 = time.time()
        if self.mock:  # offline stand-in for testing the wiring
            opt = context["my_options"][0]
            return {"action": "claim", "pick": opt["pick"], "reasoning": f"(mock) heaviest option is slot {opt['slot']}",
                    "say": "", "source": "mock", "ms": 0}
        if not self.key:
            return {"action": "fallback", "error": "no ANTHROPIC_API_KEY set"}
        body = {
            "model": self.model, "max_tokens": 300, "system": SYSTEM,
            "tools": [TOOL], "tool_choice": {"type": "tool", "name": "choose_action"},
            "messages": [{"role": "user", "content": "Current situation (JSON):\n" + json.dumps(context)}],
        }
        req = urllib.request.Request(API, data=json.dumps(body).encode(), method="POST", headers={
            "content-type": "application/json", "x-api-key": self.key, "anthropic-version": "2023-06-01"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                resp = json.loads(r.read())
        except urllib.error.HTTPError as e:
            return {"action": "fallback", "error": f"HTTP {e.code}: {e.read()[:200].decode(errors='ignore')}"}
        except Exception as e:  # timeout, no network, DNS...
            return {"action": "fallback", "error": f"{type(e).__name__}: {e}"[:200]}
        self.calls += 1
        for block in resp.get("content", []):
            if block.get("type") == "tool_use" and block.get("name") == "choose_action":
                x = block.get("input", {})
                return {"action": x.get("action", "claim"), "pick": x.get("pick"), "reasoning": x.get("reasoning", ""),
                        "say": (x.get("say_to_human") or "")[:120], "source": "claude", "ms": int((time.time() - t0) * 1000)}
        return {"action": "fallback", "error": "no tool call in response"}
