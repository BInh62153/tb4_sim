#!/usr/bin/env bash

#  TurtleBot4 Simulation Launcher
#  _____________________________________________________________
#  Usage:
#    ./tb4sim.sh sim          → Chỉ chạy Gazebo simulator
#    ./tb4sim.sh nav          → Gazebo + Nav2 + SLAM
#    ./tb4sim.sh full         → Toàn bộ stack (nav + task manager)
#    ./tb4sim.sh rviz         → Thêm RViz2 vào stack đang chạy
#    ./tb4sim.sh cli          → Mở interactive CLI
#    ./tb4sim.sh build        → Build Docker image
#    ./tb4sim.sh logs [svc]   → Xem logs
#    ./tb4sim.sh shell [svc]  → Shell vào container
#    ./tb4sim.sh stop         → Dừng tất cả
#    ./tb4sim.sh clean        → Xóa containers + volumes

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# == Colors ==
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
DIM='\033[2m'

info()    { echo -e "${CYAN}[INFO]${RESET} $1"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $1"; }
error()   { echo -e "${RED}[ERROR]${RESET} $1"; exit 1; }

# == Dependency Check ==
check_deps() {
    command -v docker >/dev/null 2>&1 || error "Chưa cài đặt Docker!"
    command -v docker-compose >/dev/null 2>&1 || \
    docker compose version >/dev/null 2>&1 || error "Chưa cài đặt Docker Compose!"
}

# == Docker Compose Helper ==
dcomp() {
    if docker compose version >/dev/null 2>&1; then
        docker compose "$@"
    else
        docker-compose "$@"
    fi
}

# == Commands ==
cmd_build() {
    info "Đang build Docker image cho TurtleBot4 Simulation..."
    # Phải truyền --profile để compose chọn được services có khai báo build:
    # (không có profile → "No services to build")
    dcomp --profile full build
}

cmd_sim() {
    info "Khởi động Gazebo Simulator (Chỉ mô phỏng)..."
    dcomp --profile sim up -d
    info "Mở log simulator..."
    dcomp logs -f simulator
}

cmd_nav() {
    info "Khởi động Simulation + SLAM Toolbox + Nav2 Stack..."
    dcomp --profile nav up -d
    info "Stack khởi động xong. Theo dõi log:"
    echo -e "  ${DIM}./tb4sim.sh logs navigation${RESET}   ← nav2"
    echo -e "  ${DIM}./tb4sim.sh logs simulator${RESET}    ← gazebo"
}

cmd_full() {
    info "Khởi động TOÀN BỘ STACK (Sim + Nav2 + SLAM + Task Manager)..."
    dcomp --profile full up -d
    info "Stack khởi động xong. Theo dõi log:"
    echo -e "  ${DIM}./tb4sim.sh logs task_manager${RESET}  ← task manager"
    echo -e "  ${DIM}./tb4sim.sh logs navigation${RESET}    ← nav2"
    echo -e "  ${DIM}./tb4sim.sh logs simulator${RESET}     ← gazebo"
}

cmd_rviz() {
    info "Khởi động thêm RViz2 visualizer..."
    xhost +local:docker
    dcomp --profile rviz up -d
}

cmd_cli() {
    info "Mở Interactive CLI điều khiển TurtleBot4..."
    dcomp up -d cli
    docker exec -it tb4_cli bash -c \
        "source /opt/ros/humble/setup.bash && python3 /ros2_ws/scripts/tb4_cli.py"
}

cmd_stop() {
    info "Dừng tất cả containers..."
    dcomp --profile "*" down
}

cmd_clean() {
    warn "Xóa containers, networks và volumes?"
    read -p "Tiếp tục? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        info "Đang tiến hành dọn dẹp..."
        
        # Thêm --profile "*" để dọn sạch tất cả các profile
        # Bỏ 2>/dev/null để thấy được log của Docker Compose
        dcomp --profile "*" down -v --remove-orphans
        
        info "Đã dọn dẹp sạch sẽ hệ thống."
    else
        info "Hủy bỏ."
    fi
}

cmd_status() {
    info "Trạng thái các dịch vụ đang chạy:"
    dcomp ps
}

cmd_logs() {
    local svc="${1:-}"
    if [ -z "$svc" ]; then
        dcomp logs --tail=100 -f
    else
        dcomp logs --tail=100 -f "$svc"
    fi
}

cmd_shell() {
    local svc="${1:-}"
    if [ -z "$svc" ]; then
        error "Vui lòng chọn service (sim, nav, task, rviz)"
    fi

    case "$svc" in
        sim|simulator)
            info "Mở shell vào simulator..."
            docker exec -it tb4_simulator bash ;;
        nav|navigation)
            info "Mở shell vào navigation..."
            docker exec -it tb4_navigation bash ;;  # FIXED: tb4_nav2 → tb4_navigation (tên đúng trong compose)
        task|task_manager)
            info "Mở shell vào task_manager..."
            docker exec -it tb4_task_manager bash ;; # <── ĐÃ SỬA: Đổi từ 'task_manager' thành 'tb4_task_manager' cho chuẩn xác với compose.yml
        rviz)
            info "Mở shell vào rviz..."
            docker exec -it tb4_rviz bash ;;
        *)
            error "Không tìm thấy service tên: $svc. Chọn (sim/nav/task/rviz)" ;;
    esac
}

cmd_watchdog() {
    info "Khởi động Watchdog theo dõi Healthcheck..."
    declare -A last_restart=()
    local cooldown_seconds=60

    while true; do
        # Tìm các container có tiền tố tb4_ đang ở trạng thái unhealthy
        unhealthy_containers=$(docker ps --filter "health=unhealthy" --format "{{.Names}}" | grep "^tb4_" || true)
        
        for container in $unhealthy_containers; do
            if [ -n "$container" ]; then
                now=$(date +%s)
                last=${last_restart[$container]:-0}
                if (( now - last < cooldown_seconds )); then
                    continue
                fi

                warn "[WATCHDOG] Phát hiện $container bị treo (unhealthy). Đang khởi động lại..."
                docker restart "$container"
                last_restart[$container]=$now
            fi
        done
        
        # Cực kỳ quan trọng: Thêm thời gian nghỉ để không chiếm dụng 100% CPU
        sleep 10
    done
}

cmd_save_map() {
    local map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    info "Đang yêu cầu slam_toolbox lưu bản đồ với tên: [ $map_name ]..."
    
    # Container tên là tb4_slam 
    # Map được lưu bởi slam_toolbox chạy trong tb4_slam
    if ! docker ps | grep -q tb4_slam; then
        error "Container slam không chạy. Hãy khởi động stack 'nav' hoặc 'full' trước."
    fi

    docker exec -it tb4_slam ros2 run nav2_map_server map_saver_cli -f "/ros2_ws/maps/$map_name"
    info "Bản đồ đã được lưu vào thư mục maps/ dưới tên: $map_name.yaml và $map_name.pgm"
}

cmd_help() {
    cat << EOF
${BOLD}SỬ DỤNG:${RESET}
  ./tb4sim.sh [lệnh] [tham số]

${BOLD}PROFILES KHỞI ĐỘNG:${RESET}
  ${GREEN}sim${RESET}              Chỉ chạy Gazebo Simulator (Không tìm đường)
  ${GREEN}nav${RESET}              Gazebo + SLAM Toolbox + Nav2 Stack
  ${GREEN}full${RESET}             Toàn bộ hệ thống (Mô phỏng + Dẫn đường + Tuần tra tự động)
  ${GREEN}rviz${RESET}             Thêm RViz2 (chạy song song với profile khác)

${BOLD}QUẢN LÝ:${RESET}
  ${YELLOW}build${RESET}            Build Docker image
  ${YELLOW}stop${RESET}             Dừng tất cả
  ${YELLOW}clean${RESET}            Xóa containers + volumes
  ${YELLOW}status${RESET}           Hiện trạng thái
  ${YELLOW}logs [svc]${RESET}       Xem logs (vd: logs simulator)
  ${YELLOW}shell [svc]${RESET}      Shell vào container (sim/nav/slam/task/rviz)

${BOLD}ĐIỀU KHIỂN:${RESET}
  ${CYAN}cli${RESET}              Mở interactive CLI điều khiển robot
  ${CYAN}save_map [tên]${RESET}   Lưu bản đồ SLAM hiện tại

${BOLD}QUICK START:${RESET}
  ${DIM}1. ./tb4sim.sh build      # lần đầu${RESET}
  ${DIM}2. ./tb4sim.sh nav        # khởi động${RESET}
  ${DIM}3. ./tb4sim.sh rviz       # xem bản đồ${RESET}
  ${DIM}4. ./tb4sim.sh cli        # điều khiển${RESET}

EOF
}

# == Dispatch ==
check_deps

COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
    build)     cmd_build ;;
    sim)       cmd_sim ;;
    nav)       cmd_nav ;;
    full)      cmd_full ;;
    rviz)      cmd_rviz ;;
    cli)       cmd_cli ;;
    stop)      cmd_stop ;;
    clean)     cmd_clean ;;
    status)    cmd_status ;;
    logs)      cmd_logs "${1:-}" ;;
    shell)     cmd_shell "${1:-}" ;;
    save_map)  cmd_save_map "${1:-}" ;;
    watchdog)  cmd_watchdog ;;
    help|*)    cmd_help ;;
esac