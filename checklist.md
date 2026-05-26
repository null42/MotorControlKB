# 知识库架构优化与用户注释功能 — 验证 Checklist

> 验证时间：2026-05-19
> 结果：31/32 通过，1 项失败

---

## 架构优化

- [x] `/api/knowledge-base/list` 返回的 Sections 包含「硬件路径」「算法路径」「MC_LIB代码实践」「高级」「参考附录」
- [x] 「参考附录」包含「控制理论」「电子基础」「电源路径」三个子 Section
- [x] hardware/algorithm 模块列表来自文件系统扫描而非硬编码
- [x] `*-assessment.md` 文件不出现在侧边栏模块列表中
- [x] `/api/knowledge-base/doc/` API 可正常加载新增三个目录下的 .md 文件
- [x] 路径穿越请求（`../`）返回 403

## 注释后端 API

- [x] `_annotations/` 目录自动创建且结构正确
- [x] `GET /api/annotations?docPath=xxx` 正确返回或返回空
- [x] `PUT /api/annotations` 保存注释后 JSON 文件正确写入
- [x] `DELETE /api/annotations` 删除注释后 JSON 文件正确更新
- [x] `GET /api/annotations` 聚合查询返回所有注释
- [x] 自由笔记 CRUD API 全流程正常

## 文档页注释交互

- [x] 文档页面每个 h2/h3 标题旁出现 📝 注释按钮
- [x] 已有注释的小节，注释按钮为高亮/实心样式
- [x] 点击注释按钮展开折叠面板，动画流畅
- [x] 新小节：面板直接进入编辑模式，有 textarea + 保存/取消按钮
- [x] 已有注释：面板先显示渲染后的内容（只读），点击「编辑」切换编辑模式
- [x] 保存成功后面板关闭，图标变高亮，显示提示
- [x] 清空内容保存后，该小节注释被删除，图标恢复空心
- [x] 修改但未保存时关闭面板，弹出确认对话框
- [x] 内容未变更时关闭面板，直接关闭不弹窗
- [x] 切换文档时若有未保存注释，弹出确认对话框

## 我的笔记页面

- [x] 侧边栏底部显示「我的笔记」导航入口
- [ ] 入口旁显示注释总数徽标 — **BUG：始终为 0**（见下方说明）
- [x] 页面 Tab 切换「章节注释」/「自由笔记」正常工作
- [x] 章节注释按知识库目录树状结构展示，顺序正确
- [x] 有注释的小节显示摘要和更新时间
- [x] 点击章节注释条目可跳转到对应文档页
- [x] 自由笔记：新建、编辑、删除全流程正常
- [x] 自由笔记按创建时间倒序排列

## 兼容性

- [x] 现有功能（学习仪表盘、灵活刷题、知识图谱）不受影响
- [x] 现有 API 返回值格式向后兼容，前端不报错

---

## 失败项详情

### [ ] 入口旁显示注释总数徽标

**文件：** `ExamDashboard/src/components/AppLayout.tsx:32-37`

**问题：** 聚合查询 `GET /api/annotations`（无 query 参数）返回格式为 `{ annotations: [...] }`，但前端解析逻辑为：

```javascript
setTotalNotes(Array.isArray(data) ? data.length : data.total ?? data.count ?? 0);
```

`Array.isArray({annotations:[...]})` 为 `false`，接着 `data.total`、`data.count` 均为 `undefined`，导致 `totalNotes` 始终为 0，徽标永不显示。

**修复：** 改为：
```javascript
setTotalNotes(data.annotations?.length || 0);
```

---

## 总结

| 分类 | 通过 | 失败 |
|------|------|------|
| 架构优化 | 6 | 0 |
| 注释后端 API | 6 | 0 |
| 文档页注释交互 | 10 | 0 |
| 我的笔记页面 | 7 | 1 |
| 兼容性 | 2 | 0 |
| **总计** | **31** | **1** |