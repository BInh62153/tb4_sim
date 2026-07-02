#!/usr/bin/env bash
# Wrap Gazebo/ROS launch with VirtualGL when USE_VIRTUALGL=true.
# Xvfb (:99) provides 2D display; VirtualGL redirects OpenGL to GPU (EGL).
set -euo pipefail

#region agent log
_dbg() {
  local msg="$1" data="$2" hid="${3:-H0}"
  printf '{"sessionId":"a3be86","timestamp":%s,"location":"vgl_launch.sh","message":"%s","data":%s,"hypothesisId":"%s"}\n' \
    "$(date +%s%3N)" "$msg" "$data" "$hid" >> /ros2_ws/.cursor/debug-a3be86.log 2>/dev/null || true
}
#endregion

USE_VGL="${USE_VIRTUALGL:-true}"
VGL_BIN="$(command -v vglrun || true)"

#region agent log
_dbg "vgl_launch entry" "{\"use_virtualgl\":\"${USE_VGL}\",\"vglrun_found\":$([ -n \"$VGL_BIN\" ] && echo true || echo false),\"display\":\"${DISPLAY:-unset}\",\"vgl_display\":\"${VGL_DISPLAY:-unset}\"}" "H1"
#endregion

if [ "$USE_VGL" = "true" ] && [ -n "$VGL_BIN" ]; then
  # VRAM-friendly setting for 4GB GPUs
  export VGL_PROBEGLX="${VGL_PROBEGLX:-0}"
  export VGL_READBACK="${VGL_READBACK:-sync}"
  export VGL_LOGO="${VGL_LOGO:-0}"
  export VGL_VERBOSE="${VGL_VERBOSE:-0}"

  # Sanity: verify EGL/GLX backend before launching Gazebo
  if vglrun glxinfo >/tmp/vgl_glxinfo.txt 2>/tmp/vgl_glxinfo.err; then
    renderer="$(grep -m1 'OpenGL renderer' /tmp/vgl_glxinfo.txt || echo 'unknown')"
    #region agent log
    _dbg "vglrun glxinfo ok" "{\"renderer\":\"${renderer//\"/\\\"}\"}" "H2"
    #endregion
  else
    err="$(head -3 /tmp/vgl_glxinfo.err 2>/dev/null | tr '\n' ' ')"
    #region agent log
    _dbg "vglrun glxinfo failed" "{\"error\":\"${err//\"/\\\"}\"}" "H2"
    #endregion
    echo "[vgl_launch] WARN: vglrun glxinfo failed — falling back to software rendering" >&2
    export LIBGL_ALWAYS_SOFTWARE=1
    exec "$@"
  fi

  echo "[vgl_launch] GPU render via VirtualGL (VGL_DISPLAY=${VGL_DISPLAY:-egl0})" >&2
  #region agent log
  _dbg "exec vglrun" "{\"cmd\":\"$*\"}" "H3"
  #endregion
  exec vglrun "$@"
fi

if [ "$USE_VGL" = "true" ] && [ -z "$VGL_BIN" ]; then
  echo "[vgl_launch] WARN: USE_VIRTUALGL=true but vglrun not found — software fallback" >&2
  #region agent log
  _dbg "vglrun missing" "{}" "H1"
  #endregion
  export LIBGL_ALWAYS_SOFTWARE=1
fi

exec "$@"
