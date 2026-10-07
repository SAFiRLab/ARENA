#!/bin/bash
# Helpers to run the testbench nodes in their own terminal and to stop them.
# Source this file, don't execute it.
#
# Terminal modes, chosen with TESTBENCH_TERMINAL (default: tmux if installed, otherwise inline):
#   tmux   : one tmux window per node, the script attaches to the session
#   xterm  : one xterm window per node (needs an X display visible on your screen)
#   inline : the output of every node is printed in the current terminal with a [name] prefix,
#            Ctrl-C stops all the nodes

UTILS_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

if [ -z "$TESTBENCH_TERMINAL" ]; then
    if command -v tmux > /dev/null; then
        TESTBENCH_TERMINAL="tmux"
    else
        TESTBENCH_TERMINAL="inline"
    fi
fi
export TESTBENCH_TERMINAL

TESTBENCH_TMUX_SESSION=${TESTBENCH_TMUX_SESSION:-testbench}
export TESTBENCH_TMUX_SESSION

# Keep the terminal open once the command exits (set to 0 to close it)
TERMINAL_HOLD=${TERMINAL_HOLD:-1}

# Processes started by the testbench scripts
TESTBENCH_PROCESS_PATTERNS=(
    "bin/[r]os2 launch arena_core costmap_3D_launch.py"
    "bin/[r]os2 launch arena_core testbench_launch.py"
    "bin/[r]os2 launch arena_core linedrone_test_node_launch.py"
    "lib/arena_core/[c]ostmap_3D_node"
    "lib/arena_core/[t]estbench_node"
    "lib/arena_core/[l]inedrone_test_node"
    "bin/[r]os2 launch arena_core benchmark_planner_launch.py"
    "lib/arena_core/[m]oar_3d_node"
    "lib/arena_core/[r]rt_star_node"
    "lib/arena_core/[s]pline_nsga2_node"
)

# Usage: run_in_terminal <title> <command>
# Every output is also appended to /tmp/<title>.log
run_in_terminal() {
    local title="$1"
    local cmd="$2"
    local log="/tmp/$title.log"

    case "$TESTBENCH_TERMINAL" in
        tmux)
            local tmux_cmd="$cmd 2>&1 | tee -ia $log"
            [ "$TERMINAL_HOLD" = "1" ] && tmux_cmd="$tmux_cmd; echo; echo '[$title exited, press Enter to close]'; read"
            local env_args=(-e "TESTBENCH_TERMINAL=$TESTBENCH_TERMINAL" -e "TESTBENCH_TMUX_SESSION=$TESTBENCH_TMUX_SESSION")
            if ! tmux has-session -t "$TESTBENCH_TMUX_SESSION" 2> /dev/null; then
                tmux new-session -d -s "$TESTBENCH_TMUX_SESSION" -n "$title" -x 200 -y 50 "${env_args[@]}" bash -c "$tmux_cmd"
                # Click on the windows names to switch between them and scroll with the wheel
                tmux set-option -t "$TESTBENCH_TMUX_SESSION" mouse on > /dev/null
            elif tmux list-windows -t "$TESTBENCH_TMUX_SESSION" -F '#W' | grep -qx "$title"; then
                tmux respawn-window -k -t "$TESTBENCH_TMUX_SESSION:$title" "${env_args[@]}" bash -c "$tmux_cmd"
            else
                tmux new-window -d -t "$TESTBENCH_TMUX_SESSION:" -n "$title" "${env_args[@]}" bash -c "$tmux_cmd"
            fi
            ;;
        xterm)
            local hold_flag=""
            [ "$TERMINAL_HOLD" = "1" ] && hold_flag="-hold"
            xterm -T "$title" -fa 'Monospace' -fs 12 -geometry 80x43+150+460 $hold_flag -e bash -c "$cmd 2>&1 | tee -ia $log" &
            ;;
        *)
            bash -c "$cmd" 2>&1 | tee -ia "$log" | sed -u "s/^/[$title] /" &
            ;;
    esac
}

# Stops every node started by the testbench scripts
stop_testbench_nodes() {
    local pattern

    echo "Stopping the testbench nodes..."

    # The launch processes forward SIGINT to their nodes
    for pattern in "${TESTBENCH_PROCESS_PATTERNS[@]}"; do
        pkill -INT -f "$pattern"
    done

    # Give them 10 s to shutdown cleanly, then force it
    for _ in $(seq 20); do
        local running=0
        for pattern in "${TESTBENCH_PROCESS_PATTERNS[@]}"; do
            pgrep -f "$pattern" > /dev/null && running=1
        done
        [ "$running" = "0" ] && break
        sleep 0.5
    done

    for pattern in "${TESTBENCH_PROCESS_PATTERNS[@]}"; do
        if pgrep -f "$pattern" > /dev/null; then
            echo "Forcing: $pattern"
            pkill -KILL -f "$pattern"
        fi
    done

    if command -v tmux > /dev/null && tmux has-session -t "$TESTBENCH_TMUX_SESSION" 2> /dev/null; then
        tmux kill-session -t "$TESTBENCH_TMUX_SESSION"
    fi

    echo "Testbench nodes stopped"
}

# Call at the end of a test script once all the nodes are started
follow_testbench_nodes() {
    local stop_script="$UTILS_DIR/kill_testbench_nodes.sh"

    case "$TESTBENCH_TERMINAL" in
        tmux)
            echo "Nodes are running in tmux session '$TESTBENCH_TMUX_SESSION', one window per node."
            echo "  Switch windows: click on their names at the bottom, or Ctrl-b n / Ctrl-b p"
            echo "  Leave without stopping the nodes: Ctrl-b d (come back with: tmux attach -t $TESTBENCH_TMUX_SESSION)"
            echo "  Stop all the nodes: $stop_script"
            if [ -t 0 ] && [ -t 1 ] && [ -z "$TMUX" ]; then
                sleep 1
                tmux attach -t "$TESTBENCH_TMUX_SESSION"
            fi
            ;;
        xterm)
            echo "Nodes are running in xterm windows. Stop all the nodes with: $stop_script"
            ;;
        *)
            echo "Nodes are running, their output is shown below and saved in /tmp/<node>.log."
            echo "Press Ctrl-C to stop all the nodes."
            trap 'trap - INT TERM; stop_testbench_nodes; exit 0' INT TERM
            wait
            # Every node exited by itself, make sure the planner restarted by the testbench is stopped too
            stop_testbench_nodes
            ;;
    esac
}
