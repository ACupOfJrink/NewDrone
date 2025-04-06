import streamlit as st

page_2 = st.Page("pages/page_2.py", title="首页")

page_1 = st.Page("pages/page_1.py", title="编队控制")

page0 = st.Page("pages/page0.py", title="队形变换")

page1 = st.Page("pages/page1.py", title="无源定位")

page2 = st.Page("pages/page2.py", title="云端模拟")

page3 = st.Page("pages/page3.py", title="避障算法")

pg = st.navigation([page_2, page_1, page0, page1, page3, page2])

pg.run()
