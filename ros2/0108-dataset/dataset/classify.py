import os
import random
import shutil

# 设置路径
current_path = os.getcwd()
images_dir = os.path.join(current_path, 'images')
labels_dir = os.path.join(current_path, 'labels')
train_dir = os.path.join(current_path, 'train')
val_dir = os.path.join(current_path, 'val')

# 创建训练集和验证集的images和labels文件夹
train_images_dir = os.path.join(train_dir, 'images')
train_labels_dir = os.path.join(train_dir, 'labels')
val_images_dir = os.path.join(val_dir, 'images')
val_labels_dir = os.path.join(val_dir, 'labels')

os.makedirs(train_images_dir, exist_ok=True)
os.makedirs(train_labels_dir, exist_ok=True)
os.makedirs(val_images_dir, exist_ok=True)
os.makedirs(val_labels_dir, exist_ok=True)

# 获取所有图片文件
image_files = [f for f in os.listdir(images_dir) if f.endswith('.jpg')]
random.shuffle(image_files)  # 随机打乱

# 计算训练集和验证集的分割点
split_point = int(len(image_files) * 0.8)

# 分割数据集
train_files = image_files[:split_point]
val_files = image_files[split_point:]

# 复制图片和标签到对应的文件夹
for file in train_files:
    # 复制图片
    shutil.copy(os.path.join(images_dir, file), os.path.join(train_images_dir, file))
    # 复制对应的标签
    label_file = file.replace('.jpg', '.txt')
    shutil.copy(os.path.join(labels_dir, label_file), os.path.join(train_labels_dir, label_file))

for file in val_files:
    # 复制图片
    shutil.copy(os.path.join(images_dir, file), os.path.join(val_images_dir, file))
    # 复制对应的标签
    label_file = file.replace('.jpg', '.txt')
    shutil.copy(os.path.join(labels_dir, label_file), os.path.join(val_labels_dir, label_file))

print("数据集分割完成！")