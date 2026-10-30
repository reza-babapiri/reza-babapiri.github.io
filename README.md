# Reza Babapiri — Engineering Portfolio v05

Complete static site, ready to publish at **https://reza-babapiri.github.io/**.

## Contents

- **13 expanded project case-study pages** across five disciplines.
- Original user-supplied photograph of the obstacle-avoiding robot at `assets/projects/robot_prototype.jpg`.
- Four real Fusion 360 CAD/CAM screenshot files (`fusion_cad.jpg`, `fusion_material.jpg`, `fusion_setup.jpg`, `fusion_toolpath.jpg`) plus a focused CAD cover.
- A small professional portrait in the About section: `assets/reza_portrait.jpg`.
- More source-derived plots plus clearly identified illustrative workflow diagrams for each case study.
- Five **updated PDF category portfolios** and an updated overview PDF in `pdfs/`.
- A link from every project to the broader site and its category PDF.
- Existing repository URL and project paths are preserved.

## Updating your live GitHub Pages site (manual, using the GitHub website)

The repository is **`reza-babapiri/reza-babapiri.github.io`** and the active branch is **main**.

**Safest recommended method (GitHub Desktop)**
1. Download this ZIP and extract to a normal local folder.
2. Open GitHub Desktop, sign in and **File → Clone repository** for `reza-babapiri/reza-babapiri.github.io` (do NOT create a new repository).
3. Open the cloned repository folder from GitHub Desktop.
4. Copy **all contents inside the extracted `site` folder** into that cloned repository folder. Allow replacement of existing files; do not place an extra parent folder above `index.html`.
5. In GitHub Desktop inspect the changed-file list. **Commit to main**, then **Push origin**.
6. Return to GitHub → Actions to verify deployment and open https://reza-babapiri.github.io/ with Ctrl+Shift+R to bypass cache.

**Browser-only alternative**
- GitHub `Add file → Upload files` supports adding new files and directories via drag/drop. To reliably update **existing** `index.html`, `styles.css`, `projects.json` and individual `projects/*.html`, GitHub Desktop is recommended. Browser upload may refuse same-path existing files. If updating manually through the browser, edit/replace existing files separately and upload the new images under `assets/`; do not remove all existing pages at once.

### Post-deployment checks
- https://reza-babapiri.github.io/ (homepage with small profile photo)
- https://reza-babapiri.github.io/projects/robot.html (actual tracked robot photo)
- https://reza-babapiri.github.io/projects/fusion.html (four original Fusion 360 screenshots)
- https://reza-babapiri.github.io/projects/buck.html (additional response and Bode plots)
- https://reza-babapiri.github.io/projects/vision.html (classical vs DeepLabv3+ results)
- https://reza-babapiri.github.io/pdfs/Reza_Babapiri_mechanical_Portfolio.pdf (PDF download)

### Accuracy and privacy
- Author says he carried out the technical implementation of the coursework; some academic deliverables were submitted with other named students. Website mentions academic submission context without taking credit for others' independent work.
- Never upload proprietary employer CAD, client files, emails, private contact lists or full unredacted university reports.
- Only original user-supplied photos, images extracted from submitted reports, selected publicly shareable scripts, and the user's own model screenshots are bundled.
- Simulations and code are described as documentary evidence **not** verified physical experiments unless hardware photos are available. Course simulations have not been rerun.
- A number of screenshots include university or educational-license UI. They are included as project records rather than certification of industrial manufacture.
- Motor V6 and planetary gearbox remain future independent projects, not completed items.

## v06 changes
- Technical ownership statement on the homepage and on every case-study page.
- Updated PDF cover credits to acknowledge personal engineering work.
- Still includes the actual tracked robot photo, author's small portrait, original Fusion 360 screenshots, and expanded data-driven figure galleries.
- All relative links and existing URLs are preserved.
