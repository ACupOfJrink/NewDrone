import streamlit as st
import numpy as np
from PIL import Image
import base64
from io import BytesIO
import pandas as pd
import time
st.set_page_config(page_title="首页", page_icon=":drone:", layout="wide")

#函数区
def image_to_base64(image):
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode()

def stream_data(_LOREM_IPSUM):
        for word in _LOREM_IPSUM.split(" "):
            yield word + " "
            time.sleep(0.02)

        for word in _LOREM_IPSUM.split(" "):
            yield word + " "
            time.sleep(0.02)      
   


#CSS样式
#加粗
custom_style="""
<style>
    .Font6Bold {
        font-weight:bold;
        font size:150px;
    }
</style>
"""

#楷体
st.markdown(
    """
    <style>
    .kai-font {
        font-family: "KaiTi", "楷体", serif;
    }
    </style>
    """,
    unsafe_allow_html=True
)

#居中
st.markdown(
    """
    <style>
    .center {
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

#楷体居中
st.markdown(
    """
    <style>
    .kai-font_center {
        font-family: "KaiTi", "楷体", serif;
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)


st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/SummaryUP.png")) + '" width="1200"></div>', unsafe_allow_html=True)
            
