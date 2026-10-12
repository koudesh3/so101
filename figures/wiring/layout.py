"""Shared by build_scene.py (in Blender) and compose.py: where the canvas sits over the scene, and
where each arm's board, servos and power supply go (an offset in mm from the follower's).

    follower   the follower's bus alone
    both       the follower's and, behind it, the leader's, each on its own USB-C port, board and supply

port: which of the Mac's USB-C ports (0 the rear one, 1 the front one). cord: which way the supply's
mains cord runs off to the wall, "front" (+y) or "side" (+x, between the two rows of servos). names:
whether its servos are tagged with their joint names, or only numbered (one row of names is enough).
mac_dy: slides the Mac along y; with two rows its two USB-C ports sit level with the gap between them.
"""
S = 2.9   # canvas px per mm along the projection, as gen_wiring.py
# Both canvases have the same shape (1480 x 1080), so the post's figure can zoom from one to the other
# filling its frame; the follower's picture sits in the two-arm one at (OX, OY) less its own.

VARIANTS = {
    "follower": dict(W=1480, H=1080, OX=1045, OY=470, inset=True,
                     arms=[dict(name="follower", dx=0, dy=0, port=1, cord="front", names=True)]),
    "both": dict(W=2080, H=1518, OX=1645, OY=749, inset=False, mac_dy=-112,
                 arms=[dict(name="follower", dx=0, dy=0, port=1, cord="front", names=False),
                       dict(name="leader", dx=0, dy=-240, port=0, cord="side", names=True)]),
    # The CH343P inset in the follower's figure: the chip close up, on a plain table, 900 x 395 (clear of the first servo's tag);
    # wiring.html places it in the follower's picture, lined up with its toggle.
    "chip": dict(W=900, H=395, OX=520, OY=131, S=28, scene="chip", grid=False, inset=False, arms=[]),
}
