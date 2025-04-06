# CT-tasks



第一次尝试使用GitHub，这是一个git-cheatsheet。

# 🚀 Git 常用命令速查表

一个简洁实用的 Git 命令手册，适用于日常开发、协作和项目管理。

---

## 🧱 初始化 & 配置

```bash
git init                          # 初始化本地仓库
git clone <仓库地址>             # 克隆远程仓库到本地
git config --global user.name "你的名字"    # 设置用户名
git config --global user.email "你的邮箱"  # 设置邮箱
```

---

## 📦 本地提交流程

```bash
git status                        # 查看当前状态
git add <文件名>                 # 添加单个文件到暂存区
git add .                         # 添加所有文件
git commit -m "提交说明"         # 提交到本地仓库
git log                           # 查看提交历史
git diff                          # 查看当前改动
```

---

## 🚀 与远程仓库交互（GitHub）

```bash
git remote add origin <远程地址>   # 添加远程仓库
git push -u origin main            # 第一次 push 建立追踪
git push                           # 后续 push
git pull                           # 拉取远程更新
git clone <远程地址>              # 克隆远程仓库
```

---

## 🌱 分支操作

```bash
git branch                         # 查看本地分支
git branch <分支名>               # 创建分支
git checkout <分支名>            # 切换分支
git checkout -b <分支名>         # 创建并切换分支
git merge <分支名>               # 合并分支
git branch -d <分支名>           # 删除分支
```

---

## 🧯 撤销与恢复

```bash
git restore <文件>                # 撤销文件修改（未 add）
git reset <文件>                  # 取消 add
git commit --amend                # 修改上一条提交信息
git reset --hard HEAD~1          # 回退上一个提交（⚠️危险）
```

---

## 🔍 日常辅助命令

```bash
git show                          # 查看提交详情
git stash                         # 暂存当前改动
git stash pop                     # 恢复上次暂存内容
git tag <标签名>                 # 打标签（如 v1.0.0）
git fetch                         # 拉取远程变更但不合并
```

---

## 🚨 常见错误处理

```bash
# 远程已有内容，push 被拒
git pull origin main --allow-unrelated-histories

# 冲突解决后继续提交
git add .
git commit -m "resolve conflict"

# 用本地版本覆盖远程（慎用）
git push -f
```

---

## 📌 总结

Git 日常操作基本流程：

```bash
git add .
git commit -m "说明"
git push
```

掌握常用分支、pull、merge，你就可以轻松应对 90% 的 Git 场景。
