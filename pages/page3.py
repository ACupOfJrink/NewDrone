import streamlit as st
from PIL import Image
import base64
from io import BytesIO

def image_to_base64(image):
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode()



# 设置网页信息  page_icons用于显示表情
st.set_page_config(page_title="空中使者——队形算法", layout="wide")


# 加载图片
image1 = Image.open(r"images\image1.png")
image2 = Image.open(r"images\image1.png")


import streamlit as st
from PIL import Image

# 设置居中样式
st.markdown(
    """
    <style>
    .center {
        display: flex;
        justify-content: center;
        align-items: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

# 加载图片


# 使用 div 和 CSS 样式居中显示图片
st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(image1) + '" width="500"></div>', unsafe_allow_html=True)

#引入空格换行
st.markdown(" ")
st.markdown('<div class="center"><img src="data:image1/png;base64,' + image_to_base64(image1) + '" width="500"></div>', unsafe_allow_html=True)
st.markdown('<div class="center"><img src="data:image2/png;base64,' + image_to_base64(image1) + '" width="500"></div>', unsafe_allow_html=True)




st.title("队形算法设计")

st.header("目标问题")
st.markdown('''假设队形变换在无障碍物情况下进行变换，则无人机p从q到的所有路线中直线路径最短，那么问题就可以将队形变化变成优化问题。该优化问题需要求解在最小时间内的队形变化方案，该优化问题实际上是一个指派问题。若编队中有 架飞机，则潜在的指派方案就有 种。目标函数为能量耗费 和使用时间 的加权平均，则优化问题的数学表达式为''')
st.latex(r'''minJ(k, \zeta)=\alpha E(k)+(1-\alpha) \textit{T}(k) \\ \begin{cases} 
\textit{E}(k) = \textit{E}(k) = \sum^{n}_{i=1} \textit{E}(k) 
 \\ 
\textit{T}(k)=max\{ t_1(k),t_2(k), \cdots , t_{n(k)} , \alpha \in [0,1]\}
 \end{cases}''')




import streamlit as st

# 使用 CSS 居中公式
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

# 使用 st.markdown() 或 st.latex() 来显示公式






# #字体尝试
# import streamlit as st

# # 设置自定义 CSS
# st.markdown(
#     """
#     <style>
#     body {
#         font-family: "KaiTi", "楷体", serif;
#     }
#     </style>
#     """,
#     unsafe_allow_html=True
# )

# # 你可以在这里添加其他组件
# st.title('这是一个使用楷体的标题')
# st.write('这段文字也将使用楷体字体')





# #独立字体设置尝试
# import streamlit as st

# # 设置字体为楷体 (KaiTi)
# st.markdown(
#     """
#     <style>
#     .kai-font {
#         font-family: "KaiTi", "楷体", serif;
#     }
#     </style>
#     """,
#     unsafe_allow_html=True
# )

# # 设置字体为宋体 (SimSun)
# st.markdown(
#     """
#     <style>
#     .song-font {
#         font-family: "SimSun", "宋体", serif;
#     }
#     </style>
#     """,
#     unsafe_allow_html=True
# )

# # 设置字体为黑体 (SimHei)
# st.markdown(
#     """
#     <style>
#     .hei-font {
#         font-family: "SimHei", "黑体", serif;
#     }
#     </style>
#     """,
#     unsafe_allow_html=True
# )

# # 用不同字体展示文本
# st.markdown('<p class="kai-font">这段文字使用楷体显示。</p>', unsafe_allow_html=True)
# st.markdown('<p class="song-font">这段文字使用宋体显示。</p>', unsafe_allow_html=True)
# st.markdown('<p class="hei-font">这段文字使用黑体显示。</p>', unsafe_allow_html=True)

# # 你也可以在其他组件中使用相同的类
# st.write('<p class="kai-font">这是楷体字体的内容！</p>', unsafe_allow_html=True)
# st.write('<p class="song-font">这是宋体字体的内容！</p>', unsafe_allow_html=True)
# st.write('<p class="hei-font">这是黑体字体的内容！</p>', unsafe_allow_html=True)

