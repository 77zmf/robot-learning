# 第16分会邀请函生成系统

[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Live-brightgreen)](https://你的用户名.github.io/这个仓库名/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> 中国化学会第35届学术年会 · 第16分会基础化学教育分会 · 邀请函生成系统

## 🌐 在线访问

**访问地址**: `https://你的用户名.github.io/这个仓库名/`

## 📋 系统说明

本系统用于生成中国化学会第35届学术年会第16分会（基础化学教育分会）的个性化邀请函。

### 核心功能
- ✅ **身份验证**：内置 220 位受邀嘉宾名单
- ✅ **一键生成**：验证通过后自动生成Word格式邀请函
- ✅ **即时下载**：生成后自动下载带个人姓名的文档
- ✅ **移动端适配**：支持手机、平板、电脑访问

### 技术栈
- 纯前端实现（HTML + CSS + JavaScript）
- 使用 [docx.js](https://github.com/dolanmiu/docx) 生成Word文档
- 使用 [FileSaver.js](https://github.com/eligrey/FileSaver.js) 实现文件下载
- 托管于 GitHub Pages

## 🚀 部署方法

### 1. Fork/创建仓库
1. 在GitHub上创建新仓库（Repository name: `invitation-system`）
2. 设置为 **Public**（GitHub Pages免费版仅支持Public仓库）

### 2. 上传文件
将以下文件上传到仓库根目录：
- `index.html`（主程序）
- `_config.yml`（配置文件，禁用Jekyll）
- `README.md`（本文件）

### 3. 启用GitHub Pages
1. 进入仓库 **Settings** → **Pages**
2. **Source** 选择 **Deploy from a branch**
3. **Branch** 选择 **main** (或 master)，文件夹选择 **/(root)**
4. 点击 **Save**
5. 等待1-2分钟，访问提供的链接

### 4. 自定义域名（可选）
如需使用自定义域名（如 `invite.chemsoc.org.cn`）：
1. 在仓库根目录创建 `CNAME` 文件
2. 文件中填写域名：`invite.chemsoc.org.cn`
3. 在DNS服务商添加CNAME记录指向 `你的用户名.github.io`

## 🔒 安全说明

- 受邀名单已嵌入前端JavaScript中进行验证
- 非名单内人员无法生成邀请函
- 所有数据均为公开会议信息，无敏感隐私数据

## 📞 联系方式

- **联系人**：魏锐
- **邮箱**：weirui131@163.com
- **会议官网**：http://www.chemsoc.org.cn/meeting/35th

## 📄 License

MIT License - 仅供中国化学会第35届学术年会第16分会使用
