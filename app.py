import streamlit as st
import time
from datetime import datetime

st.set_page_config(page_title="MAVLink 演示", page_icon="🚁")

st.title("🚁 MAVLink 心跳包演示")

# 初始化状态
if 'running' not in st.session_state:
    st.session_state.running = False
if 'count' not in st.session_state:
    st.session_state.count = 0

# 控制按钮
col1, col2 = st.columns(2)
with col1:
    if st.button("▶️ 开始发送心跳", disabled=st.session_state.running):
        st.session_state.running = True
        st.rerun()

with col2:
    if st.button("⏹️ 停止", disabled=not st.session_state.running):
        st.session_state.running = False
        st.rerun()

# 显示状态
status = "🟢 运行中" if st.session_state.running else "🔴 已停止"
st.metric("当前状态", status)

# 实时更新
if st.session_state.running:
    st.session_state.count += 1
    st.write(f"心跳包 #{st.session_state.count} 发送于 {datetime.now().strftime('%H:%M:%S')}")
    time.sleep(1)
    st.rerun()

# 显示历史
if st.session_state.count > 0:
    st.progress(min(st.session_state.count / 100, 1.0))
    st.write(f"总计发送: {st.session_state.count} 个心跳包")
