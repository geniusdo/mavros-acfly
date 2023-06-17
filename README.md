# 日志  

### 支持功能
sync  
odom receive  
add nanolog framework  

### LOG 使用方法  
首先找到 ` mavros/src/mavros_node.cpp `  
将里面的log生成地址改成想要的log生成地址  
再找到 ` mavros/CMakeLists.txt`  
`option(ARM "if ARM" ON) `  
`option(LOG "if LOG" ON) `  
如果是ARM架构处理器就写ON，如果是x86的就写OFF  
LOG 同理
