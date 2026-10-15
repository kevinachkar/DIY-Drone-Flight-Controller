# Upload this project to GitHub

Repository: https://github.com/kevinachkar/DIY-Drone-Flight-Controller

## Upload the source folders

1. Extract the ZIP on your computer.
2. Open your repository and select **Add file → Upload files**.
3. Open the extracted project folder in File Explorer.
4. Drag the folders **firmware**, **libraries**, **transmitter**, **ground-control**, **docs**, and **media** into GitHub's upload area. Upload their folders, not the ZIP and not the outer project folder.
5. Optionally upload `.gitignore` too.
6. Use commit message: **Add flight firmware, custom libraries, transmitter and PC bridge**.
7. Commit the upload.

## Update the existing README

The packaged README is a suggested replacement; the existing online README could not be retrieved during packaging.

Open README.md on GitHub and click its pencil icon. Compare your current text with the supplied README, keep any details you want, then paste or merge the supplied content. Preview it and commit with **Document drone setup, controls and current progress**.

## Add your photo

1. Place your original `IMG_8073.jpg` inside the local `media/photos/` folder.
2. Upload that folder through **Add file → Upload files** from the repository root.
3. In the README, add this outside a code block:

```markdown
![Custom DIY drone](media/photos/IMG_8073.jpg)
```

## Add flight videos

Your originals are approximately 100.2 MB and 40.4 MB in decimal units. Both exceed GitHub's normal browser repository-upload limit of 25 MiB per file.

### Recommended simple option: YouTube links

Upload the two videos to YouTube. Use **Unlisted** if you want them accessible through their links without appearing in ordinary search results.

Add the actual links under “Flight media” in README.md:

```markdown
- [Flight demonstration 1](PASTE_FIRST_VIDEO_URL_HERE)
- [Flight demonstration 2](PASTE_SECOND_VIDEO_URL_HERE)
```

Replace the labels with accurate descriptions, such as “Manual flight” or “Position-hold test”, after checking each clip.

### Alternative: GitHub-hosted video clips

For a free GitHub plan, trim/compress each clip below 10 MB. MP4 encoded with H.264 is a practical choice.

Open the README editor and try dragging a small video into the text area under “Flight media”. Wait until the upload finishes and GitHub inserts its attachment URL, then preview and commit.

If that editor does not accept the upload, use **Issues → New issue**, drag the small clip into the issue body, wait for its attachment URL, copy that URL, and paste it on its own line in the README editor. You do not need to submit the issue just to obtain the attachment URL.

Keep original large video files out of the source folders. The package includes media instructions, not downloaded copies of your photo or videos, because binary-file download failed in this session.

## Optional additions later

- Frame CAD export (STEP plus STL for printing)
- KiCad schematic and PCB project
- Wiring diagram and component list
- Tested Arduino board-core and library versions
- A license of your choice if you want to grant reuse rights

No license was selected on your behalf.

## References

- https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository
- https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/attaching-files

