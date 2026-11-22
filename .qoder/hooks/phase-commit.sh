#!/usr/bin/env bash
# Stop hook: commit the working tree as a phase checkpoint.
# Never blocks the agent: every failure is logged and the turn ends normally.
set -uo pipefail

payload=$(cat 2>/dev/null || true)
cwd=$(printf '%s' "$payload" | jq -r '.cwd // empty' 2>/dev/null || true)
repo="${QODER_PROJECT_DIR:-$cwd}"
[ -n "$repo" ] && [ -d "$repo" ] || exit 0

gitdir=$(git -C "$repo" rev-parse --absolute-git-dir 2>/dev/null) || exit 0
logfile="$gitdir/qoder-phase-commit.log"
log() { printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >>"$logfile" 2>/dev/null || true; }
notify() { jq -cn --arg m "$1" '{systemMessage:$m}' 2>/dev/null; }

# --- refuse to commit in unsafe states -------------------------------------
git -C "$repo" symbolic-ref -q HEAD >/dev/null || { log "skip: detached HEAD"; exit 0; }
for marker in MERGE_HEAD REBASE_HEAD CHERRY_PICK_HEAD REVERT_HEAD; do
  [ -e "$gitdir/$marker" ] && { log "skip: $marker present"; exit 0; }
done
if [ -d "$gitdir/rebase-merge" ] || [ -d "$gitdir/rebase-apply" ]; then
  log "skip: rebase in progress"
  exit 0
fi
[ -e "$gitdir/index.lock" ] && { log "skip: index.lock held"; exit 0; }

porcelain=$(git -C "$repo" status --porcelain 2>/dev/null) || exit 0
[ -n "$porcelain" ] || exit 0

# --- refuse to stage anything that looks like a credential ------------------
# git status collapses untracked-only directories ("?? config/"), so enumerate
# individual paths instead of parsing the status output.
changed_paths=$({
  git -C "$repo" diff --name-only HEAD 2>/dev/null
  git -C "$repo" ls-files --others --exclude-standard 2>/dev/null
} | sort -u)
if printf '%s\n' "$changed_paths" | grep -Eqi \
  '(^|/)\.env($|\.)|(^|/)id_(rsa|ed25519)|(^|/)credentials|(^|/)secrets?($|\.)|\.(pem|key|p12|pfx|keystore)$'; then
  log "skip: possible secret in working tree"
  notify "自动提交已跳过：工作区疑似包含密钥或凭据文件"
  exit 0
fi

# --- stage and commit -------------------------------------------------------
git -C "$repo" add -A 2>>"$logfile" || { log "skip: git add failed"; exit 0; }
git -C "$repo" diff --cached --quiet && { log "skip: nothing staged after add"; exit 0; }

count=$(git -C "$repo" diff --cached --name-only | wc -l | tr -d ' ')
preview=$(git -C "$repo" diff --cached --name-only | head -20 | sed 's/^/  /')
areas=$(git -C "$repo" diff --cached --name-only \
  | awk -F/ '{print (NF > 1 ? $1 : "(root)")}' \
  | sort -u \
  | awk 'NR == 1 { s = $0; next } { s = s ", " $0 } END { print substr(s, 1, 80) }')
[ -n "$areas" ] || areas="workspace"

subject="chore(auto): checkpoint $areas"
summary=$(printf '%s' "$payload" | jq -r '.last_assistant_message // empty' 2>/dev/null \
  | grep -v '^[[:space:]]*$' | head -c 400 || true)

msgfile="$gitdir/qoder-commit-msg.tmp"
{
  printf '%s\n\n' "$subject"
  printf '%s\n\n' "Auto-committed by .qoder/hooks/phase-commit.sh when the agent finished a turn."
  [ -n "$summary" ] && printf 'Last assistant message:\n%s\n\n' "$summary"
  printf 'Files (%s):\n%s\n' "$count" "$preview"
} >"$msgfile"

if ! git -C "$repo" commit -q -F "$msgfile" >>"$logfile" 2>&1; then
  rm -f "$msgfile"
  log "skip: git commit failed"
  notify "自动提交失败，详情见 $logfile"
  exit 0
fi
rm -f "$msgfile"

sha=$(git -C "$repo" rev-parse --short HEAD 2>/dev/null)
log "committed $sha: $subject ($count files)"
notify "已自动提交 $sha：$subject（$count 个文件）"
exit 0
