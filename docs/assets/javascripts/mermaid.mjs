// Keep ordinary fenced Mermaid blocks readable on GitHub and render them in MkDocs.
const blocks = document.querySelectorAll("pre code.language-mermaid");
if (blocks.length) {
  try {
    const { default: mermaid } = await import(
      "https://cdn.jsdelivr.net/npm/mermaid@11.12.2/dist/mermaid.esm.min.mjs"
    );
    const nodes = Array.from(blocks, (code) => {
      const pre = code.parentElement;
      pre.textContent = code.textContent;
      pre.classList.add("mermaid");
      return pre;
    });
    mermaid.initialize({ startOnLoad: false, securityLevel: "strict" });
    await mermaid.run({ nodes });
  } catch (error) {
    console.error("Mermaid diagram rendering failed:", error);
  }
}
