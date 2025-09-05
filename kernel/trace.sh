#!/bin/bash

# 设置日志文件和行数限制
LOG_FILE="vnic.log"
MAX_LINES=10000

# 清理现有日志文件
: > "$LOG_FILE"

# 启动tail进程在后台运行
tail -f /var/log/syslog > "$LOG_FILE" &
TAIL_PID=$!

# 设置退出时清理函数
cleanup() {
    echo "停止tail进程 (PID $TAIL_PID)"
    kill -TERM "$TAIL_PID" 2>/dev/null
    wait "$TAIL_PID" 2>/dev/null
    echo "完成! 最终行数: $(wc -l < "$LOG_FILE")"
    exit 0
}

# 捕获退出信号
trap cleanup SIGINT SIGTERM EXIT

# 主监控循环
while sleep 1; do
    CURRENT_LINES=$(wc -l < "$LOG_FILE")
    
    if [ "$CURRENT_LINES" -ge "$MAX_LINES" ]; then
        echo "行数已达到 $CURRENT_LINES，超过上限 $MAX_LINES"
        cleanup
    else
        echo -ne "当前行数: $CURRENT_LINES/$MAX_LINES\r"
    fi
done