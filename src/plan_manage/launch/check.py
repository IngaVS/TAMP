import os

def file_exists(file_path):
    return os.path.exists(file_path)

# 示例使用
l=[]
for i in range(1,15001):
    prefix = "/home/han/learning_code/data_maze/e"+str(i)
    file_path = prefix+"/path1.dat"
    if file_exists(file_path):
       a=1
    else:
        l.append(i)
        print("文件不存在: ",file_path)
print('files: ',len(l))