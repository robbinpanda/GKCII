# MD 转 PDF 技能：pandoc + xelatex 中文文档生成

## 用途
将 Markdown 文件转换为 PDF 文档，**特别针对含中文字符、表格、代码块的文档**。

## 核心命令模板

```bash
# 基础命令
pandoc "input.md" -o "output.pdf" \
  --pdf-engine=xelatex \
  -V mainfont="Noto Sans CJK SC" \
  -V monofont="Noto Sans Mono CJK SC" \
  -V geometry:margin=1cm \
  -V fontsize=9pt \
  -V geometry:a4paper \
  --include-in-header=/path/to/header.tex

# 横向排版（适合宽表格）
pandoc "input.md" -o "output.pdf" \
  --pdf-engine=xelatex \
  -V mainfont="Noto Sans CJK SC" \
  -V monofont="Noto Sans Mono CJK SC" \
  -V geometry:margin=0.8cm \
  -V fontsize=7pt \
  -V geometry:a4paper,landscape \
  --include-in-header=/path/to/header.tex
```

## 关键经验

### 1. 中文字体必须配置

```latex
-V mainfont="Noto Sans CJK SC"
-V monofont="Noto Sans Mono CJK SC"
```

**错误示例**：不指定中文字体 → 所有中文显示为方块或空白。

### 2. 中文段落换行：必须加 CJK linebreak 配置

```latex
\XeTeXlinebreaklocale "zh"
\XeTeXlinebreakskip = 0pt plus 0.2em
```

**问题现象**：长段落文字在右边界被截断，部分内容消失（如"AprilTag视觉检"后直接跳到"PID控制器"）。

**根本原因**：xelatex 默认不会在中文汉字之间自动换行，只有遇到空格或标点才会换行。当一行全是中文字符时，超出边界就会被截断。

### 3. 表格溢出：三种解决方案（按优先级）

#### 方案 A：缩短 MD 中的表格内容（推荐）
- 将长英文论文名称改为缩写（如 `Coverage_Control_UAVs_UGVs_Altitude` → `Coverage Control (Altitude)`）
- 缩短"说明"列的描述文字
- 减少表格列数或合并列

#### 方案 B：使用横向排版 + 小字体
```bash
-V geometry:a4paper,landscape
-V fontsize=7pt
-V geometry:margin=0.8cm
```

#### 方案 C：LaTeX 表格缩放（慎用，可能与其他包冲突）
```latex
\usepackage{adjustbox}
\let\oldtabular\tabular
\let\endoldtabular\endtabular
\renewenvironment{tabular}{\begin{adjustbox}{max width=\textwidth}\begin{oldtabular}}{\end{oldtabular}\end{adjustbox}}
```
⚠️ **注意**：`adjustbox` 包裹 `longtable` 会导致编译错误 `! Missing \endgroup inserted.`。

### 4. 行距控制

```latex
\renewcommand{\arraystretch}{1.05}  % 表格行距（1.0=默认，1.2=较宽松）
```

### 5. 标准 header.tex 模板

```latex
\usepackage{longtable,booktabs}
\usepackage{fvextra}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{breaklines,commandchars=\\\{\}}
\setlength{\emergencystretch}{3em}
\providecommand{\tightlist}{\setlength{\itemsep}{0pt}\setlength{\parskip}{0pt}}
\renewcommand{\arraystretch}{1.05}
\setlength{\LTleft}{0pt}
\setlength{\LTright}{0pt}

% 中文换行配置（关键！）
\XeTeXlinebreaklocale "zh"
\XeTeXlinebreakskip = 0pt plus 0.2em
```

### 6. 不同文档类型的推荐参数

| 文档类型 | 页面方向 | 字体大小 | 边距 | 说明 |
|----------|----------|----------|------|------|
| 普通文章 | portrait | 9pt | 1.0cm | 标准设置 |
| 宽表格/评分表 | landscape | 7-8pt | 0.8cm | 表格内容多的文档 |
| 代码文档 | portrait | 8pt | 1.0cm | 代码块需要 `fvextra` |
| 详细分析报告 | landscape | 7pt | 0.8cm | 有大量段落和列表 |

### 7. 验证方法：PDF 转图片检查

```bash
# 生成 PNG 预览
pdftoppm -png -r 200 "output.pdf" "preview"

# 检查关键页面
ls preview-*.png
```

**检查重点**：
1. 右边缘是否有文字被截断
2. 表格行是否有重叠
3. 长段落是否完整换行（无内容丢失）
4. 底部是否有内容被截断

### 8. 常见问题速查

| 问题现象 | 原因 | 解决方案 |
|----------|------|----------|
| 中文显示为方块 | 未指定中文字体 | 加 `-V mainfont="Noto Sans CJK SC"` |
| 段落文字在边界截断 | 未配置 CJK linebreak | header.tex 中加 `\XeTeXlinebreaklocale "zh"` |
| 表格超出右边界 | 列内容太长 | 缩短 MD 内容 + landscape + 小字体 |
| 表格行重叠 | 行高太小 | 增加 `\arraystretch` |
| 代码块溢出 | 未启用 breaklines | header 中加 `fvextra` 和 `breaklines` |
| `Missing \endgroup` | adjustbox 包裹 longtable | 移除 adjustbox，改用缩短内容 |
| 警告 Missing character | 等宽字体缺字 | 加 `-V monofont="Noto Sans Mono CJK SC"` |

### 9. 工作流程（收到 md→pdf 任务时）

1. **分析 MD 内容**：检查是否有宽表格（论文名称、长描述）、长段落、代码块
2. **预估排版需求**：
   - 有宽表格 → 横向A4 + 7-8pt + 0.8cm边距
   - 纯文字文章 → 纵向A4 + 9pt + 1cm边距
3. **编写 header.tex**：必须包含 CJK linebreak 配置
4. **生成 PDF**：使用 pandoc + xelatex
5. **转换为 PNG 预览**：`pdftoppm -png -r 200`
6. **视觉检查**：检查右边界截断、表格溢出、段落完整性
7. **修复问题**：
   - 截断 → 缩短 MD 内容 / 换横向 / 调小字体
   - 重叠 → 增加行距 / 调整边距
8. **重新生成并验证**

## 完整示例脚本

```bash
#!/bin/bash
# md2pdf.sh - Markdown to PDF converter for Chinese documents

INPUT="$1"
OUTPUT="$2"
ORIENTATION="${3:-portrait}"  # portrait or landscape
FONT_SIZE="${4:-9pt}"

# 创建临时 header
cat > /tmp/md2pdf_header.tex << 'HEADER'
\usepackage{longtable,booktabs}
\usepackage{fvextra}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{breaklines,commandchars=\\\{\}}
\setlength{\emergencystretch}{3em}
\providecommand{\tightlist}{\setlength{\itemsep}{0pt}\setlength{\parskip}{0pt}}
\renewcommand{\arraystretch}{1.05}
\setlength{\LTleft}{0pt}
\setlength{\LTright}{0pt}
\XeTeXlinebreaklocale "zh"
\XeTeXlinebreakskip = 0pt plus 0.2em
HEADER

if [ "$ORIENTATION" = "landscape" ]; then
    pandoc "$INPUT" -o "$OUTPUT" \
        --pdf-engine=xelatex \
        -V mainfont="Noto Sans CJK SC" \
        -V monofont="Noto Sans Mono CJK SC" \
        -V geometry:margin=0.8cm \
        -V fontsize="$FONT_SIZE" \
        -V geometry:a4paper,landscape \
        --include-in-header=/tmp/md2pdf_header.tex
else
    pandoc "$INPUT" -o "$OUTPUT" \
        --pdf-engine=xelatex \
        -V mainfont="Noto Sans CJK SC" \
        -V monofont="Noto Sans Mono CJK SC" \
        -V geometry:margin=1cm \
        -V fontsize="$FONT_SIZE" \
        -V geometry:a4paper \
        --include-in-header=/tmp/md2pdf_header.tex
fi

echo "Generated: $OUTPUT"
```

## 使用方式

```bash
# 普通文档
./md2pdf.sh input.md output.pdf

# 含宽表格的文档
./md2pdf.sh input.md output.pdf landscape 7pt

# 验证
pdftoppm -png -r 200 output.pdf preview
```
