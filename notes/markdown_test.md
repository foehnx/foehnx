# Markdown Test Suite

## Overview

This document consolidates all markdown rendering test cases for the foehnx project. It serves as a comprehensive test suite to validate markdown rendering capabilities and ensure all features work correctly with the rendering stack.

## Technology Stack

### Rendering Libraries

- **[marked.js](https://marked.js.org/)** - Fast markdown parser and compiler built for speed
  - [GitHub Repository](https://github.com/markedjs/marked)
  - [Documentation](https://marked.js.org/)

- **[KaTeX](https://katex.org/)** - Fast math typesetting library for the web
  - [GitHub Repository](https://github.com/KaTeX/KaTeX)
  - [Supported Functions](https://katex.org/docs/supported.html)

- **[Highlight.js](https://highlightjs.org/)** (likely) - Syntax highlighting for code blocks
  - [GitHub Repository](https://github.com/highlightjs/highlight.js)

---

## Test Section 1: Content Without Headings

**Purpose:** Tests markdown rendering when no heading structure is present, ensuring proper parsing of basic content at the document root level.

**Source:** `noheadings.md`

bla bla
* list item
* list item

---

## Test Section 2: Basic Text Formatting

**Purpose:** Tests all standard inline text formatting options including bold, italic, strikethrough, inline code, highlighting, and special characters.

### Basic Styles

This is **bold text** and this is *italic text*. You can also use __bold__ and _italic_ syntax.

Combine them for ***bold and italic*** text.

Here is ~~strikethrough~~ text.

This is `inline code` within a sentence.

Use ==highlighted text== for emphasis.

Superscript: x^2^ and subscript: H~2~O.

### HTML Tag Support

<b>bold</b>
<i>italic</i>

<kbd>Ctrl</kbd>+<kbd>C</kbd> keyboard styling

<mark>This text is marked/highlighted.</mark>

<sub>subscript</sub> and <sup>superscript</sup> via HTML.

---

## Test Section 3: Lists

**Purpose:** Tests both ordered and unordered list rendering, including nested list structures and various list syntax styles.

### Unordered List

- Item one
  - Nested item A
    - Deep nested item
  - Nested item B
- Item two
- Item three

### Alternate Syntax

* list
* item

- other list
- other item

### Ordered List

1. First step
2. Second step
   1. Sub-step one
   2. Sub-step two
3. Third step

### Task Lists

- [x] Write introduction
- [x] Add code examples
- [x] Include KaTeX formulas
- [ ] Review and proofread
- [ ] Publish document

---

## Test Section 4: Links and Images

**Purpose:** Tests various link formats including inline, reference-style, autolinks, and image embedding.

[Visit GitHub](https://github.com)

Autolink: <https://www.example.com>

Reference-style link: [Markdown Guide][1]

Internal link: [no headings](notes/noheadings.md)

![Placeholder Image](https://placehold.co/300x200)

[1]: https://www.markdownguide.org

---

## Test Section 5: Code Blocks

**Purpose:** Tests syntax highlighting for multiple programming languages and special code block types like diffs.

### Inline Code

Inline: Use `printf("hello")` in C.

### C++

```cpp
int main() {
    return 0;
}
```

### Python

```python
def fibonacci(n: int) -> list[int]:
    """Generate the first n Fibonacci numbers."""
    seq = [0, 1]
    for _ in range(2, n):
        seq.append(seq[-1] + seq[-2])
    return seq[:n]

print(fibonacci(10))
```

### Rust

```rust
fn main() {
    let primes: Vec<u32> = (2..100)
        .filter(|&n| (2..=(n as f64).sqrt() as u32).all(|d| n % d != 0))
        .collect();
    println!("{:?}", primes);
}
```

### JSON

```json
{
  "name": "markdown-showcase",
  "version": "1.0.0",
  "features": ["tables", "katex", "code blocks"]
}
```

### Diff

```diff
- const old = "removed line";
+ const new = "added line";
```

---

## Test Section 6: Tables

**Purpose:** Tests table rendering with various alignment options (left, center, right) and inline formatting within cells.

| Feature        | Supported | Notes                     |
| :------------- | :-------: | ------------------------: |
| Bold           |    Yes    |         `**text**`        |
| Italic         |    Yes    |          `*text*`         |
| Strikethrough  |    Yes    |        `~~text~~`         |
| Tables         |    Yes    |     Pipe syntax           |
| KaTeX          |    Yes    | Inline and display mode   |

---

## Test Section 7: Mathematical Expressions (KaTeX)

**Purpose:** Tests KaTeX integration for rendering complex mathematical notation, both inline and in display mode. Covers various mathematical domains: calculus, linear algebra, physics, statistics, and complex analysis.

### Inline Math

The quadratic formula is $x = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}$ and Euler's identity is $e^{i\pi} + 1 = 0$.

The derivative of $f(x) = x^n$ is $f'(x) = nx^{n-1}$.

Simple inline variable: $p_x$ just like this.

### Display Math

The Gaussian integral:

$$
\int_{-\infty}^{\infty} e^{-x^2} \, dx = \sqrt{\pi}
$$

Maxwell's equations in differential form:

$$
\begin{aligned}
\nabla \cdot \mathbf{E} &= \frac{\rho}{\varepsilon_0} \\
\nabla \cdot \mathbf{B} &= 0 \\
\nabla \times \mathbf{E} &= -\frac{\partial \mathbf{B}}{\partial t} \\
\nabla \times \mathbf{B} &= \mu_0 \mathbf{J} + \mu_0 \varepsilon_0 \frac{\partial \mathbf{E}}{\partial t}
\end{aligned}
$$

A matrix equation:

$$
\begin{bmatrix}
a_{11} & a_{12} & a_{13} \\
a_{21} & a_{22} & a_{23} \\
a_{31} & a_{32} & a_{33}
\end{bmatrix}
\begin{bmatrix} x \\ y \\ z \end{bmatrix}
=
\begin{bmatrix} b_1 \\ b_2 \\ b_3 \end{bmatrix}
$$

Test with align environment:

$$
\begin{align}
x &=3 \\
\text{bla bla} &= 42
\end{align}
$$

Taylor series expansion:

$$
f(x) = \sum_{n=0}^{\infty} \frac{f^{(n)}(a)}{n!}(x - a)^n
$$

Schrödinger equation:

$$
i\hbar \frac{\partial}{\partial t} \Psi(\mathbf{r}, t) = \hat{H} \Psi(\mathbf{r}, t)
$$

Bayes' theorem:

$$
P(A \mid B) = \frac{P(B \mid A) \, P(A)}{P(B)}
$$

Cauchy integral formula:

$$
f(a) = \frac{1}{2\pi i} \oint_\gamma \frac{f(z)}{z - a} \, dz
$$

Navier-Stokes equation:

$$
\rho \left( \frac{\partial \mathbf{v}}{\partial t} + \mathbf{v} \cdot \nabla \mathbf{v} \right) = -\nabla p + \mu \nabla^2 \mathbf{v} + \mathbf{f}
$$

### Piecewise Functions

$$
|x| = \begin{cases}
x  & \text{if } x \geq 0 \\
-x & \text{if } x < 0
\end{cases}
$$

---

## Test Section 8: Blockquotes

**Purpose:** Tests blockquote rendering including nested quotes, multi-paragraph quotes, and quotes containing other markdown elements.

> This is a blockquote.
>
> It can span multiple paragraphs.

> **Nested blockquotes:**
>
> > "Mathematics is the queen of the sciences."
> > — Carl Friedrich Gauss

> Blockquote with other elements:
>
> - List item inside a quote
> - Another item
>
> Code inside a quote: `E = mc^2`

---

## Test Section 9: Horizontal Rules

**Purpose:** Tests different horizontal rule syntax variations.

Three different syntaxes:

---

***

___

Alternative syntax (not standard):

===

---

## Test Section 10: Footnotes

**Purpose:** Tests footnote references and definitions, including footnotes with mathematical expressions.

Here is a sentence with a footnote[^1]. And another one[^2].

[^1]: This is the first footnote with additional detail.
[^2]: This is the second footnote referencing $E = mc^2$.

---

## Test Section 11: Definition Lists

**Purpose:** Tests definition list syntax for term-definition pairs.

Markdown
: A lightweight markup language for creating formatted text.

KaTeX
: A fast math typesetting library for the web.

LaTeX
: A document preparation system for high-quality typesetting.

---

## Test Section 12: Abbreviations

**Purpose:** Tests abbreviation expansion on hover or with tooltips.

The HTML specification is maintained by the W3C.

*[HTML]: Hyper Text Markup Language
*[W3C]: World Wide Web Consortium

---

## Test Section 13: Collapsible Sections (HTML Details)

**Purpose:** Tests HTML5 `<details>` and `<summary>` elements for creating expandable/collapsible content sections, including accordion-style groups with the `name` attribute.

### Basic Collapsed Section

<details>
<summary>Click to expand hidden content</summary>

This content is hidden by default. It can contain:

- Lists
- **Formatted text**
- Even math: $\sum_{k=1}^{n} k = \frac{n(n+1)}{2}$

```python
print("Hidden code block!")
```

</details>

### Standalone Section

<details>
<summary>
A Summary
</summary>
followed by plenty of detail.
</details>

### Accordion Group (Exclusive Sections)

<details name="ac">
<summary>
A
</summary>
Super Stuff!
</details>

<details name="ac">
<summary>
B
</summary>
Awesome Stuff!
</details>

<details name="ac">
<summary>
C
</summary>
Incredible Stuff!
</details>

---

## Test Section 14: Emoji

**Purpose:** Tests GitHub-Flavored Markdown emoji shortcodes.

:rocket: :sparkles: :white_check_mark: :warning: :bulb:

---

## Test Section 15: Escaping Special Characters

**Purpose:** Tests proper escaping of markdown special characters to display them literally.

Use backslash to escape: \*not italic\*, \`not code\`, \[not a link\].

Literal pipe in text: \|

