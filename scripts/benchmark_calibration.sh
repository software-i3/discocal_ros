#!/usr/bin/env bash
# Run one both-camera calibration against a rosbag and measure it.
#
#   bash scripts/benchmark_calibration.sh \
#       --bag /output/calibration_set_big.bag --rate 1 --budget 100
#
# The script starts a private ROS master (unless one is already running), the
# node, and the action client; it replays the bag only after the client has
# reported COLLECTING, so the feed is not consumed while the node is idle. It
# samples /proc once a second for thread count and RSS and wraps the node in
# /usr/bin/time -v for authoritative user/system CPU and peak RSS.
#
# Everything it writes lands in --report-dir (default
# <output-dir>/benchmark_<timestamp>/): node/client/bag logs, CPU samples,
# node_time.txt, and report.txt. The calibration artifacts themselves are in
# the run directory the node names (also printed at the end).
#
# Run this inside the ROS Melodic container, with devel/setup.bash sourced.
set -u

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STARTED_AT="$(date '+%Y-%m-%d %H:%M:%S')"
HAVE_GNU_TIME=0

BAG=""
RATE="1"
CAMERA="both"
BUDGET="100"
TIMEOUT="1800"
LOOPS="1"
CONFIG="$PKG_DIR/config/rig_example_both.yaml"
OUTPUT_DIR="/output"
REPORT_DIR=""
ACTION="/discocal_ros/calibrate_camera"
SAMPLE_INTERVAL="1"
ASSUME_MASTER="0"

usage() {
    cat <<EOF
Usage: $0 --bag FILE [options]

  --bag FILE            rosbag to replay (required)
  --camera NAME         goal camera: both, left or right (default: both)
  --rate FLOAT          rosbag play rate; 0 = as fast as possible (default: 1)
  --budget N            accepted frames per camera (default: 100)
  --timeout SECONDS     client wait for a result (default: 1800)
  --loops N             times to replay the bag if it ends first; 0 = until
                        the client finishes (default: 1)
  --config FILE         rig YAML (default: config/rig_example_both.yaml)
  --output-dir DIR      value for ~output_dir (default: /output)
  --report-dir DIR      where logs/report go (default: <output-dir>/benchmark_<time>)
  --action NAME         action server (default: $ACTION)
  --sample-interval S   /proc sampling period (default: 1)
  --assume-master       do not start roscore; use the one already running
  -h, --help            this text
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --bag) BAG="$2"; shift 2 ;;
        --camera) CAMERA="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --budget) BUDGET="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --loops) LOOPS="$2"; shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --report-dir) REPORT_DIR="$2"; shift 2 ;;
        --action) ACTION="$2"; shift 2 ;;
        --sample-interval) SAMPLE_INTERVAL="$2"; shift 2 ;;
        --assume-master) ASSUME_MASTER="1"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [ -z "$BAG" ] || [ ! -f "$BAG" ]; then
    echo "FAIL: --bag must name an existing file" >&2
    exit 2
fi
if [ ! -f "$CONFIG" ]; then
    echo "FAIL: config not found: $CONFIG" >&2
    exit 2
fi
# This script starts the node with rosrun, which -- unlike roslaunch -- never
# parses the launch files. A malformed launch file would therefore pass the
# benchmark and fail only in deployment, so parse them here. XML comments may
# not contain "--", and roslaunch's parser is strict about it.
for lf in "$PKG_DIR"/launch/*.launch; do
    [ -e "$lf" ] || continue
    if command -v python >/dev/null 2>&1; then
        python -c 'import sys, xml.dom.minidom; xml.dom.minidom.parse(sys.argv[1])' "$lf" \
            2>/dev/null || { echo "FAIL: $lf is not well-formed XML; roslaunch will refuse it" >&2; exit 2; }
    fi
done

if [ -z "$REPORT_DIR" ]; then
    REPORT_DIR="$OUTPUT_DIR/benchmark_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$REPORT_DIR" || { echo "FAIL: cannot create $REPORT_DIR" >&2; exit 2; }

NODE_LOG="$REPORT_DIR/node.log"
CLIENT_LOG="$REPORT_DIR/client.log"
BAG_LOG="$REPORT_DIR/bag.log"
ROSCORE_LOG="$REPORT_DIR/roscore.log"
SAMPLES="$REPORT_DIR/samples.txt"
TIME_FILE="$REPORT_DIR/node_time.txt"
REPORT="$REPORT_DIR/report.txt"

ROSCORE_PID=""
TIME_PID=""
NODE_PID=""
CLIENT_PID=""
BAG_PID=""
SAMPLER_PID=""
CLIENT_RC=""

cleanup() {
    # Never leave a play/client/node running after a failure or Ctrl-C.
    [ -n "$CLIENT_PID" ] && kill "$CLIENT_PID" 2>/dev/null
    [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null
    [ -n "$SAMPLER_PID" ] && kill "$SAMPLER_PID" 2>/dev/null
    if [ -n "$NODE_PID" ]; then
        kill -INT "$NODE_PID" 2>/dev/null
        for _ in $(seq 1 20); do
            kill -0 "$NODE_PID" 2>/dev/null || break
            sleep 0.25
        done
        kill -TERM "$NODE_PID" 2>/dev/null
    fi
    if [ -n "$ROSCORE_PID" ]; then
        kill "$ROSCORE_PID" 2>/dev/null
    fi
}
trap 'cleanup' EXIT INT TERM

fail() {
    echo "FAIL: $*" >&2
    if [ -f "$NODE_LOG" ]; then
        echo "--- node.log (tail) ---" >&2
        tail -20 "$NODE_LOG" >&2
    fi
    exit 1
}

# ---------------------------------------------------------------------------
# Master
# ---------------------------------------------------------------------------
if pgrep -f 'discocal_ros/discocal_ros_node' >/dev/null 2>&1; then
    fail "a discocal_ros_node is already running; stop it so the benchmark measures the node it starts"
fi

if [ "$ASSUME_MASTER" = "1" ]; then
    rosnode list >/dev/null 2>&1 || fail "--assume-master was given but no master is reachable"
else
    if rosnode list >/dev/null 2>&1; then
        echo "using the already-running ROS master"
    else
        roscore >"$ROSCORE_LOG" 2>&1 &
        ROSCORE_PID=$!
        for _ in $(seq 1 100); do
            rosnode list >/dev/null 2>&1 && break
            kill -0 "$ROSCORE_PID" 2>/dev/null || fail "roscore exited; see $ROSCORE_LOG"
            sleep 0.2
        done
        rosnode list >/dev/null 2>&1 || fail "roscore did not become reachable"
    fi
fi

# ---------------------------------------------------------------------------
# Node (under /usr/bin/time for user/sys CPU and peak RSS)
# ---------------------------------------------------------------------------
echo "starting node (config $CONFIG, output $OUTPUT_DIR)"
# The image does not always ship GNU time; the /proc sampler below covers the
# same numbers at 1 s resolution, and the node writes precise getrusage totals
# to performance.yaml either way.
if [ -x /usr/bin/time ]; then
    /usr/bin/time -v -o "$TIME_FILE" \
        rosrun discocal_ros discocal_ros_node \
        _rig_config:="$CONFIG" _output_dir:="$OUTPUT_DIR" \
        >"$NODE_LOG" 2>&1 &
    TIME_PID=$!
    HAVE_GNU_TIME=1
else
    rosrun discocal_ros discocal_ros_node \
        _rig_config:="$CONFIG" _output_dir:="$OUTPUT_DIR" \
        >"$NODE_LOG" 2>&1 &
    TIME_PID=$!
    HAVE_GNU_TIME=0
fi

# rosrun execs the binary, so $TIME_PID is the wrapper and the node is its
# child. Find the real pid for /proc sampling.
for _ in $(seq 1 100); do
    NODE_PID="$(pgrep -f 'discocal_ros/discocal_ros_node' | head -1 || true)"
    [ -n "$NODE_PID" ] && break
    kill -0 "$TIME_PID" 2>/dev/null || fail "the node exited during startup; see $NODE_LOG"
    sleep 0.1
done
[ -n "$NODE_PID" ] || fail "could not find the node process"
echo "node pid $NODE_PID"

for _ in $(seq 1 200); do
    rostopic list 2>/dev/null | grep -q "calibrate_camera/goal" && break
    kill -0 "$NODE_PID" 2>/dev/null || fail "the node exited during startup; see $NODE_LOG"
    sleep 0.2
done
rostopic list 2>/dev/null | grep -q "calibrate_camera/goal" || \
    fail "the action server never appeared; see $NODE_LOG"
echo "action server ready"

# ---------------------------------------------------------------------------
# /proc sampler
# ---------------------------------------------------------------------------
(
    printf '# elapsed_s utime_ticks stime_ticks threads rss_kb\n' >"$SAMPLES"
    start_ns=$(date +%s%N)
    while kill -0 "$NODE_PID" 2>/dev/null; do
        now_ns=$(date +%s%N)
        if [ -r "/proc/$NODE_PID/stat" ]; then
            stat_line=$(cat "/proc/$NODE_PID/stat" 2>/dev/null) || break
            rest="${stat_line##*) }"
            # shellcheck disable=SC2086
            set -- $rest   # $1 = state (field 3); utime is field 14
            utime="${12:-0}"
            stime="${13:-0}"
            threads="${18:-0}"
            rss_kb=$(awk '/^VmRSS:/ {print $2}' "/proc/$NODE_PID/status" 2>/dev/null)
            printf '%d %s %s %s %s\n' \
                "$(( (now_ns - start_ns) / 1000000000 ))" \
                "$utime" "$stime" "$threads" "${rss_kb:-0}" >>"$SAMPLES"
        fi
        sleep "$SAMPLE_INTERVAL"
    done
) &
SAMPLER_PID=$!

# ---------------------------------------------------------------------------
# Client, then feed
# ---------------------------------------------------------------------------
echo "sending $CAMERA goal (budget $BUDGET)"
python -u "$PKG_DIR/scripts/calibrate_client.py" \
    --camera "$CAMERA" --budget "$BUDGET" --timeout "$TIMEOUT" --action "$ACTION" \
    >"$CLIENT_LOG" 2>&1 &
CLIENT_PID=$!

for _ in $(seq 1 300); do
    grep -q "COLLECTING" "$CLIENT_LOG" 2>/dev/null && break
    kill -0 "$CLIENT_PID" 2>/dev/null || break
    sleep 0.2
done
if ! grep -q "COLLECTING" "$CLIENT_LOG" 2>/dev/null; then
    wait "$CLIENT_PID" 2>/dev/null
    CLIENT_RC=$?
    fail "the client never reported COLLECTING (exit $CLIENT_RC); see $CLIENT_LOG"
fi
echo "client is collecting; starting bag playback at rate $RATE"

CLIENT_START=$SECONDS
PLAYS=0
play_bag() {
    rosbag play "$BAG" --rate "$RATE" --delay 1 --quiet >"$BAG_LOG" 2>&1 &
    BAG_PID=$!
    PLAYS=$((PLAYS + 1))
}
play_bag

WAIT_LIMIT=$((TIMEOUT + 60))
while :; do
    if ! kill -0 "$CLIENT_PID" 2>/dev/null; then
        wait "$CLIENT_PID" 2>/dev/null
        CLIENT_RC=$?
        break
    fi
    if [ $((SECONDS - CLIENT_START)) -ge "$WAIT_LIMIT" ]; then
        CLIENT_RC=124
        kill "$CLIENT_PID" 2>/dev/null
        wait "$CLIENT_PID" 2>/dev/null
        break
    fi
    if [ -n "$BAG_PID" ] && ! kill -0 "$BAG_PID" 2>/dev/null; then
        wait "$BAG_PID" 2>/dev/null
        BAG_PID=""
        if [ "$LOOPS" = "0" ] || [ "$PLAYS" -lt "$LOOPS" ]; then
            echo "bag ended after $PLAYS pass(es); replaying"
            play_bag
        fi
    fi
    sleep 0.5
done
CLIENT_END=$SECONDS

if [ -n "$BAG_PID" ]; then
    kill "$BAG_PID" 2>/dev/null
    wait "$BAG_PID" 2>/dev/null
    BAG_PID=""
fi

# Let the sampler record the shutdown, then stop the node and reap the timer.
sleep 0.2
[ -n "$SAMPLER_PID" ] && kill "$SAMPLER_PID" 2>/dev/null
SAMPLER_PID=""
kill -INT "$NODE_PID" 2>/dev/null
for _ in $(seq 1 40); do
    kill -0 "$NODE_PID" 2>/dev/null || break
    sleep 0.25
done
kill -TERM "$NODE_PID" 2>/dev/null
NODE_PID=""
wait "$TIME_PID" 2>/dev/null
TIME_PID=""
[ -n "$ROSCORE_PID" ] && kill "$ROSCORE_PID" 2>/dev/null && ROSCORE_PID=""

WALL_SECONDS=$((CLIENT_END - CLIENT_START))
BAG_BYTES=$(stat -c '%s' "$BAG" 2>/dev/null || echo 0)
BAG_SHA=$(sha256sum "$BAG" 2>/dev/null | awk '{print $1}')
BAG_SHA=${BAG_SHA:-unknown}
NPROC=$(nproc)
KERNEL=$(uname -r)

time_field() {
    # /usr/bin/time -v writes "\tKey: value"; take the first match.
    awk -v k="$1" 'index($0, k) { sub(/^.*: /, ""); print; exit }' "$TIME_FILE" 2>/dev/null
}
CPU_USER=$(time_field "User time (seconds)")
CPU_SYS=$(time_field "System time (seconds)")
CPU_PCT=$(time_field "Percent of CPU this job got")
ELAPSED_TEXT=$(time_field "Elapsed (wall clock) time")
PEAK_RSS=$(time_field "Maximum resident set size")

: "${CPU_USER:=0}"; : "${CPU_SYS:=0}"; : "${CPU_PCT:=0}"
: "${ELAPSED_TEXT:=unknown}"; : "${PEAK_RSS:=0}"

# /proc stat times are in clock ticks; CLK_TCK is 100 on Linux/x86 but read it.
TICKS=$(getconf CLK_TCK 2>/dev/null || echo 100)
SAMPLE_SUMMARY=$(awk -v tick="$TICKS" '
    NR > 1 {
        if ($4 > max_threads) max_threads = $4
        if ($5 > max_rss) max_rss = $5
        last_u = $2; last_s = $3; last_t = $1
    }
    END {
        printf "%.2f %.2f %d %d %d", last_u/tick, last_s/tick, max_threads+0, max_rss+0, last_t+0
    }' "$SAMPLES" 2>/dev/null)
SAMPLE_USER=$(echo "$SAMPLE_SUMMARY" | awk '{print $1+0}')
SAMPLE_SYS=$(echo "$SAMPLE_SUMMARY" | awk '{print $2+0}')
SAMPLE_THREADS=$(echo "$SAMPLE_SUMMARY" | awk '{print $3+0}')
SAMPLE_RSS=$(echo "$SAMPLE_SUMMARY" | awk '{print $4+0}')
SAMPLE_LAST=$(echo "$SAMPLE_SUMMARY" | awk '{print $5+0}')

if [ "$HAVE_GNU_TIME" != "1" ]; then
    # Fall back to the sampler for process CPU; it is the same clock
    # (utime/stime from /proc), just sampled rather than read at exit.
    CPU_USER="$SAMPLE_USER"
    CPU_SYS="$SAMPLE_SYS"
    CPU_PCT="not measured"
    ELAPSED_TEXT="not measured"
    PEAK_RSS="$SAMPLE_RSS"
    CPU_SOURCE="sampled /proc at ${SAMPLE_INTERVAL}s"
else
    CPU_SOURCE="getrusage via /usr/bin/time -v"
fi

if [ "$WALL_SECONDS" -gt 0 ]; then
    TOTAL_CPU=$(awk -v u="$CPU_USER" -v s="$CPU_SYS" 'BEGIN {printf "%.2f", u+s}')
    PCT_ONE=$(awk -v c="$TOTAL_CPU" -v w="$WALL_SECONDS" 'BEGIN {printf "%.1f", 100*c/w}')
    PCT_ALL=$(awk -v c="$TOTAL_CPU" -v w="$WALL_SECONDS" -v n="$NPROC" \
        'BEGIN {printf "%.1f", (n>0 ? 100*c/(w*n) : 0)}')
else
    TOTAL_CPU="0"; PCT_ONE="0"; PCT_ALL="0"
fi

if [ "$CLIENT_RC" = "0" ]; then
    VERDICT="PASS (client exit 0)"
else
    VERDICT="FAIL (client exit $CLIENT_RC)"
fi

RUN_DIR=$(awk '/^  run_directory/ {print $2; exit}' "$CLIENT_LOG" 2>/dev/null)

{
    echo "discocal_ros calibration benchmark"
    echo "=================================="
    echo ""
    echo "verdict:          $VERDICT"
    echo "started:          $STARTED_AT"
    echo "wall_seconds:     $WALL_SECONDS (client start -> result/timeout)"
    echo "bag:              $BAG ($BAG_BYTES bytes)"
    echo "bag_sha256:       $BAG_SHA"
    echo "bag_rate:         $RATE"
    echo "bag_passes:       $PLAYS"
    echo "goal_camera:      $CAMERA"
    echo "budget_per_cam:   $BUDGET"
    echo "config:           $CONFIG"
    echo "run_directory:    ${RUN_DIR:-(none)}"
    echo ""
    echo "cpu ($CPU_SOURCE):"
    echo "  user_seconds:   $CPU_USER"
    echo "  system_seconds: $CPU_SYS"
    echo "  total_seconds:  $TOTAL_CPU"
    echo "  percent_one_core: $PCT_ONE"
    echo "  percent_all_cores ($NPROC): $PCT_ALL"
    echo "  time_reported_percent: $CPU_PCT"
    echo ""
    if [ "$HAVE_GNU_TIME" = "1" ]; then
        echo "cpu (sampled /proc at ${SAMPLE_INTERVAL}s):"
        echo "  user_seconds:   $SAMPLE_USER"
        echo "  system_seconds: $SAMPLE_SYS"
        echo "  last_sample_s:  $SAMPLE_LAST"
        echo ""
    fi
    echo "memory and threads:"
    echo "  peak_rss_kb (process): $PEAK_RSS"
    echo "  peak_rss_kb (sampled): $SAMPLE_RSS"
    echo "  peak_threads (sampled): $SAMPLE_THREADS"
    echo ""
    echo "host:"
    echo "  cpus:           $NPROC"
    echo "  kernel:         $KERNEL"
    echo "  ros_distro:     ${ROS_DISTRO:-unknown}"
    echo "  opencv:         $(python -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo unknown)"
    echo ""
    echo "elapsed (time -v): $ELAPSED_TEXT"
    echo ""
    echo "artifacts:"
    if [ -n "$RUN_DIR" ] && [ -d "$RUN_DIR" ]; then
        (cd "$RUN_DIR" && ls -l)
    else
        echo "  (the client reported no run directory)"
    fi
    echo ""
    echo "logs: $REPORT_DIR"
} >"$REPORT"

cat "$REPORT"

echo ""
echo "client.log tail:"
tail -30 "$CLIENT_LOG"

if [ "$CLIENT_RC" = "0" ]; then
    exit 0
fi
exit 1
