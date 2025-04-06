import streamlit as st
import numpy as np
from PIL import Image
import base64
from io import BytesIO
import pandas as pd
import time

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

st.title('无人机编队控制与算法对比')
st.divider()
st.header(':gray[STEP1一致性理论与传统方法的基本简介]')

st.markdown('在无人机编队控制方面，主流算法包括人工势场(APF)算法、强化学习、虚拟结构法(Virtual Structure)等。我们将人工势场法、虚拟结构法与项目提出的基于一致性理论的无人机编队控制方式进行对比。')
st.markdown('')
st.markdown('基于一致性理论的无人机编队控制方式，通过无人机之间的局部信息交换来实现系统的全局协调。每个无人机通过与邻居无人机交换信息（如位置、速度）来调整自己的行为，最终实现一致性。这种方法确保所有无人机的行为最终趋向一致，保证了对无人机整体队形控制的稳定性。一致性理论通过局部控制确保每个无人机都与邻近无人机保持协调，并能精确控制无人机之间的相对位置。因此，在多个无人机之间一致性较好时，队形保持较为稳定，能够精确实现所需的队形（如矩形、五边形等）。即使在动态变化的环境中，通过调整局部规则，队形的精度和稳定性较高，这种稳定性在无人机数量增多的时候体现得更加明显。')
st.markdown('')
st.markdown('基于人工势场法的无人机编队控制方式，通过模拟目标无人机的吸引力和其他无人机的排斥力。无人机的控制是通过计算目标点与当前点之间的吸引力以及目标无人机和其他无人机之间的排斥力来确定的。最终，所有力的合成决定了无人机的运动。由于局部势场的计算，人工势场法容易受到无人机分布和目标位置的影响，可能会出现局部极小值。这使得无人机在一些情况下可能无法找到通往目标的最优路径，从而导致编队队形发生偏移或变形，尤其在复杂环境下，队形的精度较差。')
st.markdown('')
st.markdown('基于虚拟结构法的无人机编队控制方法，通过构建虚拟结构来模拟编队中各个无人机之间的相互作用和约束，将无人机编队视为一个虚拟结构，设定每个无人机为“结构体”或“质点”，它们之间的相互作用可以视为弹簧、力学连接等。对于每对相邻的无人机，可以定义一个虚拟弹簧模型来描述它们之间的相对位移和相对速度，用“弹簧力”模拟无人机之间的距离约束，“阻尼力”来模拟无人机之间的相对速度约束。设定编队控制的目标函数，最小化无人机之间的相对距离，以确保编队的形状稳定。')
st.divider()

st.header(':gray[STEP2算法对比与结果解读]')
FormationControl1,FormationControl2 = st.columns([0.5,0.5])
with FormationControl1:
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/FormationControlComparation1.png")) + '" width="600"></div>', unsafe_allow_html=True)
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/FormationControlComparation2.png")) + '" width="600"></div>', unsafe_allow_html=True)
with FormationControl2:
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/FormationControlComparation3.png")) + '" width="600"></div>', unsafe_allow_html=True)
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/FormationControlComparation4.png")) + '" width="600"></div>', unsafe_allow_html=True)
st.markdown('')
'''将三种算法进行对比，控制无人机保持一定队形实现编队转弯、直线飞行、爬升。通过模拟控制不同架数的无人机的编队飞行，对比它们的运行时间，发现基于一致性理论的无人机编队控制算法在无人机数量较多的时候可以以更短的时间实现对无人机的编队控制。这说明我们提出的基于一致性理论的无人机编队控制方式的计算复杂度更低，在控制大规模无人机系统实现编队时，它的运行速度以及对队形保持的稳定性都会更优。人工势场法在进行编队控制时可能会陷入局部最优，这使得人工势场法难以非常精确地控制无人机的整体队形，而基于一致性理论的无人机编队控制方法能够精确地控制无人机的整体队形。'''
st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/FormationControlComparation3.png")) + '" width="1200"></div>', unsafe_allow_html=True)
FormationControldf = pd.DataFrame(data = 
[[90.68,	130.96,	206.69,	322.05,	402.86,	452.11,	491.36],
[101.63,	121.63,	173.42,	284.56,	321.32,	342.31,	351.23],
[115.3,	123.67,	197.93,	289.32,	389.65,	486.33,	501.23]],

        columns = [3, 5, 7, 10, 15, 20, 25],
        index = ['人工势场法', '一致性理论', '虚拟结构法'])    
FCL, FCM, FCR = st.columns([0.23,0.9,0.1])
with FCM:
    FormationControldf    
