# 日志  

### 支持功能
sync  
odom receive  
add nanolog framework  

### LOG 使用方法  
在launch 文件里有生成log的地址，将其改成你自己的地址
再找到 ` mavros/CMakeLists.txt`  
`option(ARM "if ARM" ON) `  
`option(LOG "if LOG" ON) `  
如果是ARM架构处理器就写ON，如果是x86的就写OFF  
LOG 同理

使用方法在orig_readme.md
