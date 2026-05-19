# GitHub upload — step by step (zero experience)

You only upload the folder: **`Motor_4_integration`**  
Do **NOT** upload the whole `4_motors` folder (it has Eclipse `.metadata` — huge and useless on GitHub).

---

## Choose ONE method

| Method | Difficulty | Best if |
|--------|------------|---------|
| **A — GitHub Desktop** | Easiest | You want clicks, no commands |
| **B — Git commands** | Medium | You installed Git and like terminal |

---

# METHOD A — GitHub Desktop (recommended for you)

## Step 1 — Install GitHub Desktop

1. Open browser: https://desktop.github.com/
2. Download **GitHub Desktop for Windows**
3. Install → open it
4. Sign in with the **same GitHub account** you created

## Step 2 — Create empty repo on GitHub website

1. Go to https://github.com
2. Click **+** (top right) → **New repository**
3. Fill in:
   - **Repository name:** `stm32-pipe-inspection-robot`  
     *(or any name, no spaces — use dashes)*
   - **Description:** `STM32 multi-sensor pipe robot with Ethernet dashboard and Kalman odometry`
   - **Public** ✓
   - **Do NOT** check "Add a README" (you already have one in your folder)
4. Click **Create repository**

## Step 3 — Add your project folder in GitHub Desktop

1. In GitHub Desktop: **File → Add local repository**
2. Click **Choose…** and select:
   ```
   C:\Users\Arshman Hassan\Desktop\4_motors\Motor_4_integration
   ```
3. If it says "not a git repository" → click **create a repository** here (same folder)

## Step 4 — First commit

1. You will see a list of files on the left (thousands is OK if Debug was ignored — `.gitignore` hides build files)
2. **Uncheck** anything inside `Debug/` or `Release/` if it still appears (should not, thanks to `.gitignore`)
3. Bottom left **Summary:** type: `Initial commit - pipe inspection robot firmware`
4. Click **Commit to main**

## Step 5 — Publish to GitHub

1. Click **Publish repository** (top)
2. Name: same as website (`stm32-pipe-inspection-robot`)
3. **Keep code public**
4. Publish

Done. Open your repo URL in browser — your code is online.

## Step 6 — Edit README on GitHub (optional)

1. On GitHub website → your repo → **README.md** → pencil icon **Edit**
2. Replace `Your Name`, email, Fiverr link at bottom
3. **Commit changes**

---

# METHOD B — Git in PowerShell (after installing Git)

## Step 1 — Install Git

1. https://git-scm.com/download/win
2. Install (Next, Next — defaults OK)
3. **Close and reopen** PowerShell or Cursor terminal

Check:
```powershell
git --version
```

## Step 2 — Create repo on GitHub (same as Method A Step 2)

## Step 3 — Commands (copy one block at a time)

Open PowerShell:

```powershell
cd "C:\Users\Arshman Hassan\Desktop\4_motors\Motor_4_integration"
git init
git add .
git status
```

You should **not** see `Debug/` in the list much (gitignore).

```powershell
git commit -m "Initial commit - pipe inspection robot firmware"
```

Link to GitHub (replace YOUR_USERNAME and REPO_NAME):

```powershell
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git
git push -u origin main
```

First push asks for login:
- Use **GitHub username**
- Password = **Personal Access Token** (not your GitHub password)

### How to get Personal Access Token

1. GitHub → profile picture → **Settings**
2. Left bottom: **Developer settings**
3. **Personal access tokens** → **Tokens (classic)** → **Generate new token**
4. Note: `upload code`, check **repo**
5. Copy token — paste when Git asks for password

---

# After upload — make portfolio attractive

## On GitHub repo page (Settings → General)

- **Description:** paste one line from README
- **Website:** (later: Notion or Fiverr link)
- **Topics (tags):** `stm32` `embedded-systems` `iot` `kalman-filter` `w5500` `mpu6050` `sensor-fusion`

## Add 3 screenshots

1. Create folder on PC: `Motor_4_integration\docs\images\`
2. Put 3 PNG photos: dashboard, hardware, wiring
3. Commit again in GitHub Desktop with message `Add demo images`

## Record 2-minute video

- Phone video of: power on → browser dashboard → sensors updating
- Upload to YouTube (Unlisted)
- Add link in README: `## Demo video` → your YouTube URL

---

# What NOT to upload

- `Debug/` folder (build — huge)
- `Release/`
- Parent folder `4_motors\.metadata\` (Eclipse only)
- Passwords, WiFi keys, personal IPs if secret

---

# Troubleshooting

| Problem | Fix |
|---------|-----|
| Push too large / slow | Make sure `.gitignore` exists; delete `Debug` from commit |
| "Repository not found" | Wrong repo URL or not logged in |
| All files gray in Desktop | Click "Show ignored files" — ignore Debug |
| README looks ugly | Normal — GitHub renders `.md` nicely on repo home page |

---

# Next: Fiverr

After GitHub is live, put the repo link in Fiverr gig description.

See your mentor/chat for Fiverr gig text copy-paste.
