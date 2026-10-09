#!/usr/bin/env bash
# 现场一条命令诊断影子：健康原因 + 适配器拒绝日志 + 门控原因 + 控制话题拓扑 + 最近采集报告。
#
# 用法（机器人静止即可，全部只读）：
#   bash scripts/onsite_diagnose.sh
#
# 退出码：0 = 影子健康且控制话题上没有影子发布者；非 0 = 存在问题（输出里会指出）。
# 不会启动、停止或修改任何实车节点。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
SHADOW_LOG_DIR="${SHADOW_LOG_DIR:-log/shadow}"

set +u
source /opt/ros/humble/setup.bash
[[ -f "${REPO_ROOT}/install/setup.bash" ]] && source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"

RC=0

echo "########## 1/5 健康通道（为什么 healthy=False）##########"
python3 scripts/onsite_health_dump.py --duration 4 || RC=$?

echo
echo "########## 2/5 适配器最近的拒绝原因 ##########"
LATEST_LOG="$(ls -dt "${SHADOW_LOG_DIR}"/*/launch.log 2>/dev/null | head -1 || true)"
if [[ -n "${LATEST_LOG}" && -f "${LATEST_LOG}" ]]; then
  echo "日志: ${LATEST_LOG}"
  grep -E "rejected|unhealthy|healthy again|degraded to zero|no TF|pairing" "${LATEST_LOG}" \
    | tail -12 || echo "  （没有相关记录）"
else
  echo "  没有找到 log/shadow/*/launch.log：如果影子由其它方式启动，请把它的输出贴出来。"
fi

echo
echo "########## 3/5 门控最近的原因（csv 尾部）##########"
LATEST_CSV="$(ls -t "${SHADOW_LOG_DIR}"/shadow_*.csv 2>/dev/null | head -1 || true)"
if [[ -n "${LATEST_CSV}" && -f "${LATEST_CSV}" ]]; then
  echo "csv: ${LATEST_CSV}"
  { head -1 "${LATEST_CSV}"; tail -5 "${LATEST_CSV}"; } | column -s, -t 2>/dev/null \
    || { head -1 "${LATEST_CSV}"; tail -5 "${LATEST_CSV}"; }
else
  echo "  没有找到 log/shadow/shadow_*.csv（门控未启动或日志目录不同）"
fi

echo
echo "########## 4/5 控制话题拓扑（发布者 vs 订阅者）##########"
python3 scripts/onsite_control_audit.py || RC=2

echo
echo "########## 5/5 最近一次现场采集报告 ##########"
LATEST_REPORT="$(ls -t log/onsite/*/report.txt 2>/dev/null | head -1 || true)"
if [[ -n "${LATEST_REPORT}" && -f "${LATEST_REPORT}" ]]; then
  echo "报告: ${LATEST_REPORT}"
  sed -n '1,40p' "${LATEST_REPORT}"
else
  echo "  没有找到 log/onsite/*/report.txt；先运行：python3 scripts/onsite_inspect.py --duration 20"
fi

echo
if [[ "${RC}" -eq 0 ]]; then
  echo "== 诊断结论：影子健康，且控制话题上没有影子发布者 =="
else
  echo "== 诊断结论：存在问题（rc=${RC}）；把以上全部输出回传 =="
fi
exit "${RC}"
