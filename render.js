(function () {
  "use strict";

  function escapeHtml(str) {
    return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  }

  // --- Marked setup ---
  const renderer = new marked.Renderer();
  const headings = [];
  const default_file = "README.md";

  renderer.heading = function ({ tokens, depth, text: rawText }) {
    const text = this.parser.parseInline(tokens);
    const clean = rawText.replace(/[\s]+/g, "-").replace(/[^\w-]/g, "").toLowerCase();
    const id = clean || "heading-" + headings.length;
    if (depth <= 3) {
      headings.push({ id, text: rawText, html: text, depth });
    }
    return `<h${depth} id="${id}">${text}</h${depth}>`;
  };

  // Rewrite internal .md links and anchor-only links for hash routing
  const origLink = renderer.link.bind(renderer);
  renderer.link = function (linkData) {
    const href = linkData.href || "";
    // Internal .md link (not starting with http/https/mailto/#)
    if (href.endsWith(".md") && !href.match(/^(https?:|mailto:)/)) {
      linkData.href = "#" + href;
    }
    // Anchor-only link: #something -> #currentFile#something
    if (href.startsWith("#") && currentFile) {
      linkData.href = "#" + currentFile + href;
    }
    const html = origLink(linkData);
    // External links open in new tab
    if (href.match(/^https?:/)) {
      return html.replace("<a ", '<a target="_blank" rel="noopener" ');
    }
    return html;
  };

  marked.use(markedKatex({ throwOnError: false }));
  marked.use(
    markedEmoji.markedEmoji({
      emojis: {
        heart: "fa-solid fa-heart",
        tada: "fa-solid fa-tada",
        rocket: "fa-solid fa-rocket",
        sparkles: "fa-solid fa-sparkles",
        white_check_mark: "fa-solid fa-check",
        warning: "fa-solid fa-triangle-exclamation",
        bulb: "fa-solid fa-lightbulb",
        microscope: "fa-solid fa-microscope",
        robot: "fa-solid fa-robot",
        computer: "fa-solid fa-computer",
        mortar_board: "fa-solid fa-user-graduate",
        school: "fa-solid fa-school",
        chalkboard: "fa-solid fa-chalkboard-user",
        git: "fa-brands fa-git-alt",
        github: "fa-brands fa-github",
      },
      renderer: (token) => `<i class="${token.emoji}"></i>`,
    })
  );

  // Custom extensions for mark, superscript, subscript
  marked.use({
    extensions: [
      {
        name: 'mark',
        level: 'inline',
        start(src) { return src.match(/==/)?.index; },
        tokenizer(src) {
          const rule = /^==([^=]+)==/;
          const match = rule.exec(src);
          if (match) {
            return {
              type: 'mark',
              raw: match[0],
              text: this.lexer.inlineTokens(match[1])
            };
          }
        },
        renderer(token) {
          return `<mark>${this.parser.parseInline(token.text)}</mark>`;
        }
      },
      {
        name: 'superscript',
        level: 'inline',
        start(src) { return src.match(/\^/)?.index; },
        tokenizer(src) {
          const rule = /^\^([^^]+)\^/;
          const match = rule.exec(src);
          if (match) {
            return {
              type: 'superscript',
              raw: match[0],
              text: match[1]
            };
          }
        },
        renderer(token) {
          return `<sup>${token.text}</sup>`;
        }
      },
      {
        name: 'subscript',
        level: 'inline',
        start(src) { return src.match(/~(?!~)/)?.index; },
        tokenizer(src) {
          const rule = /^~([^~]+)~(?!~)/;
          const match = rule.exec(src);
          if (match) {
            return {
              type: 'subscript',
              raw: match[0],
              text: match[1]
            };
          }
        },
        renderer(token) {
          return `<sub>${token.text}</sub>`;
        }
      }
    ]
  });

  marked.setOptions({
    renderer,
    gfm: true,
    breaks: false,
  });

  // --- Parse URL hash: first # = file, second # = anchor ---
  function parseHash() {
    const raw = window.location.hash.slice(1); // remove leading #
    if (!raw) return { file: default_file, anchor: "" };
    // Split on first # within the hash
    const idx = raw.indexOf("#");
    if (idx === -1) {
      return { file: raw || default_file, anchor: "" };
    }
    return {
      file: raw.slice(0, idx) || default_file,
      anchor: raw.slice(idx + 1),
    };
  }

  // --- Render markdown file ---
  let currentFile = "";

  async function loadPage() {
    const { file, anchor } = parseHash();
    const article = document.getElementById("article");

    if (file !== currentFile) {
      currentFile = file;
      try {
        const resp = await fetch(file);
        if (!resp.ok) throw new Error(`${resp.status} ${resp.statusText}`);
        let md = await resp.text();

        // Strip YAML frontmatter
        md = md.replace(/^---[\s\S]*?---\n*/, "");

        // Parse markdown (marked-katex-extension handles math)
        headings.length = 0;
        let html = marked.parse(md);

        // Sanitize
        html = DOMPurify.sanitize(html, {
          ADD_TAGS: ["details", "summary", "mark", "kbd", "input",
            "math", "semantics", "mrow", "mi", "mo", "mn", "msup", "msub",
            "mfrac", "mover", "munder", "mtable", "mtr", "mtd", "mstyle",
            "mspace", "mtext", "annotation", "mpadded", "menclose", "msqrt",
            "svg", "path", "line", "rect"],
          ADD_ATTR: ["open", "name", "type", "checked", "disabled", "dir",
            "align", "fetchpriority", "style", "aria-hidden", "encoding",
            "xmlns", "displaystyle", "scriptlevel", "columnalign",
            "rowspacing", "columnspacing", "fence", "stretchy", "symmetric",
            "mathvariant", "accent", "accentunder", "lspace", "rspace",
            "linethickness", "d", "viewBox", "fill", "stroke", "stroke-width",
            "width", "height", "preserveAspectRatio", "separator",
            "minsize", "maxsize", "rowalign"],
          ALLOW_DATA_ATTR: true,
        });

        // Highlight code blocks
        const tmp = document.createElement("div");
        tmp.innerHTML = html;
        tmp.querySelectorAll("pre code").forEach((block) => {
          const classes = block.className || "";
          const langMatch = classes.match(/language-(\w+)/);
          if (langMatch && hljs.getLanguage(langMatch[1])) {
            block.innerHTML = hljs.highlight(block.textContent, { language: langMatch[1] }).value;
          }
        });
        html = tmp.innerHTML;

        article.innerHTML = html;
        buildTOC();
        highlightMenuLink(file);
      } catch (err) {
        article.innerHTML = `<h1>Not Found</h1><p>Could not load <code>${escapeHtml(file)}</code>.</p><p>${escapeHtml(err.message)}</p>`;
        headings.length = 0;
        buildTOC();
      }
    }

    // Scroll to anchor
    if (anchor) {
      requestAnimationFrame(() => {
        const target = document.getElementById(anchor) ||
          document.getElementById(anchor.toLowerCase());
        if (target) target.scrollIntoView({ behavior: "smooth" });
      });
    } else if (file !== currentFile) {
      window.scrollTo(0, 0);
    }
  }

  // --- TOC ---
  function buildTOC() {
    const list = document.getElementById("toc-list");
    list.innerHTML = "";
    const h1Count = headings.filter((h) => h.depth === 1).length;
    const topDepth = h1Count === 1 ? 2 : 1;
    const subDepth = topDepth + 1;
    headings
      .filter((h) => h.depth === topDepth || h.depth === subDepth)
      .forEach((h) => {
        const li = document.createElement("li");
        li.className = h.depth === subDepth ? "toc-sub" : "toc-top";
        const a = document.createElement("a");
        // Construct full hash with file + anchor
        a.href = "#" + currentFile + "#" + h.id;
        a.innerHTML = h.html;
        a.addEventListener("click", (e) => {
          e.preventDefault();
          const target = document.getElementById(h.id);
          if (target) target.scrollIntoView({ behavior: "smooth" });
          // On mobile, close TOC after clicking
          if (window.innerWidth < 1200) {
            toggleToc(false);
          }
        });
        li.appendChild(a);
        list.appendChild(li);
      });
  }

  // --- Highlight active menu link ---
  function highlightMenuLink(file) {
    document.querySelectorAll("#sidebar a").forEach((a) => {
      const href = a.getAttribute("href");
      const linkFile = href === "#" ? default_file : href.slice(1);
      a.classList.toggle("active", linkFile === file);
    });
  }

  // --- Sidebar & TOC toggle logic ---
  const sidebar = document.getElementById("sidebar");
  const toc = document.getElementById("toc");
  const overlay = document.getElementById("overlay");
  const content = document.getElementById("content");

  function isWide() {
    return window.innerWidth >= 1200;
  }

  function toggleSidebar(force) {
    if (isWide()) {
      const closed = typeof force === "boolean" ? !force : !sidebar.classList.contains("closed");
      sidebar.classList.toggle("closed", closed);
      sidebar.classList.toggle("open", !closed);
      content.classList.toggle("sidebar-closed", closed);
    } else {
      const open = typeof force === "boolean" ? force : !sidebar.classList.contains("open");
      sidebar.classList.toggle("open", open);
      overlay.classList.toggle("active", open || toc.classList.contains("open"));
      if (!open && !toc.classList.contains("open")) overlay.classList.remove("active");
    }
  }

  function toggleToc(force) {
    if (isWide()) {
      const closed = typeof force === "boolean" ? !force : !toc.classList.contains("closed");
      toc.classList.toggle("closed", closed);
      toc.classList.toggle("open", !closed);
      content.classList.toggle("toc-closed", closed);
    } else {
      const open = typeof force === "boolean" ? force : !toc.classList.contains("open");
      toc.classList.toggle("open", open);
      overlay.classList.toggle("active", open || sidebar.classList.contains("open"));
      if (!open && !sidebar.classList.contains("open")) overlay.classList.remove("active");
    }
  }

  // --- Theme toggle ---
  function setTheme(theme) {
    document.documentElement.setAttribute("data-theme", theme);
    localStorage.setItem("foehnx-theme", theme);
    // Swap hljs stylesheet
    const hljsLink = document.getElementById("hljs-theme");
    hljsLink.href = theme === "latte"
      ? "https://cdn.jsdelivr.net/npm/highlight.js@11.10.0/styles/catppuccin-latte.min.css"
      : "https://cdn.jsdelivr.net/npm/highlight.js@11.10.0/styles/catppuccin-mocha.min.css";
  }

  function toggleTheme() {
    const current = document.documentElement.getAttribute("data-theme");
    setTheme(current === "mocha" ? "latte" : "mocha");
  }

  // --- Event listeners ---
  document.getElementById("menu-toggle").addEventListener("click", () => toggleSidebar());
  document.getElementById("toc-toggle").addEventListener("click", () => toggleToc());
  document.getElementById("theme-toggle").addEventListener("click", toggleTheme);
  overlay.addEventListener("click", () => {
    toggleSidebar(false);
    toggleToc(false);
    overlay.classList.remove("active");
  });

  // Close mobile sidebar on link click
  sidebar.addEventListener("click", (e) => {
    if (e.target.tagName === "A" && !isWide()) {
      toggleSidebar(false);
    }
  });

  window.addEventListener("hashchange", () => {
    const { file } = parseHash();
    if (file !== currentFile) {
      currentFile = ""; // force reload
    }
    loadPage();
  });

  // Handle resize: reset sidebar states
  window.addEventListener("resize", () => {
    if (isWide()) {
      overlay.classList.remove("active");
      if (!sidebar.classList.contains("closed")) {
        sidebar.classList.add("open");
      }
      if (!toc.classList.contains("closed")) {
        toc.classList.add("open");
      }
    } else {
      sidebar.classList.remove("open");
      toc.classList.remove("open");
      content.classList.remove("sidebar-closed", "toc-closed");
    }
  });

  // --- Init ---
  const savedTheme = localStorage.getItem("foehnx-theme") || "mocha";
  setTheme(savedTheme);

  // On wide screens, default open
  if (isWide()) {
    sidebar.classList.add("open");
    toc.classList.add("open");
  }

  loadPage();
})();
