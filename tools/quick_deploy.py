#!/usr/bin/env python3
"""Persist a robot-only SDK/ROS environment in a managed Bash startup block.

Uses only Python's standard library. Never starts a Driver, SDK probe or robot
command. Passwords are obtained interactively and are never command arguments.
"""

import argparse
import contextlib
import fcntl
import getpass
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile

BEGIN = '# >>> hikrobot arm quick deploy >>>'
END = '# <<< hikrobot arm quick deploy <<<'
HEADERS = (
    'MP_Public_API.h', 'MP_Public_Struct.h', 'MP_Public_Error.h',
    'MP_Public_Defines.h', 'MP_HAR_API.h', 'MP_HAR_Struct.h',
    'MP_Motion_API.h', 'MP_Motion_Struct.h', 'MP_IO_API.h', 'MP_IO_Struct.h',
)
LIB_REL = Path('Libs/Linux/LX64/64')
SKIP_DIRS = {'.git', '.cache', '.codex', '.aws', '.ros', '__pycache__',
             'node_modules', 'build', 'install', 'log', 'Trash'}


class SetupError(Exception):
    """An actionable message that must not include password values."""


def clean_path(value):
    if any(c in str(value) for c in ('\n', '\r', '\x00')):
        raise SetupError('路径不能包含换行或NUL字符。')
    return Path(value).expanduser().resolve()


def validate_sdk(root):
    root = clean_path(root)
    missing = [h for h in HEADERS
               if not (root / 'Include/Public' / h).is_file() or
               not os.access(root / 'Include/Public' / h, os.R_OK)]
    if missing:
        raise SetupError('SDK缺少或无法读取头文件: ' + ', '.join(missing))
    library = root / LIB_REL / 'libMPSDK_PUBLIC_COMPAT.so'
    try:
        with library.open('rb') as stream:
            header = stream.read(20)
    except OSError:
        raise SetupError('无法读取SDK主库: ' + str(library)) from None
    # ELF64, little-endian, ET_DYN, EM_X86_64, matching the current vendor shim.
    if (len(header) < 20 or header[:6] != b'\x7fELF\x02\x01' or
            struct.unpack_from('<HH', header, 16) != (3, 62)):
        raise SetupError('SDK主库必须为Linux x86_64的ELF64动态库。')
    return root


def discover_sdks(search_roots, hints=(), max_depth=6):
    """Bounded, read-only search. Do not follow directory symlinks recursively."""
    found = set()
    for hint in hints:
        if hint:
            with contextlib.suppress(SetupError, OSError):
                found.add(validate_sdk(hint))
    seen_roots = set()
    for value in search_roots:
        try:
            base = clean_path(value)
        except (SetupError, OSError):
            continue
        if base in seen_roots or not base.is_dir():
            continue
        seen_roots.add(base)
        for directory, children, _ in os.walk(base, followlinks=False):
            here = Path(directory)
            depth = len(here.relative_to(base).parts)
            with contextlib.suppress(SetupError, OSError):
                if (here / 'Include/Public/MP_Public_API.h').is_file():
                    found.add(validate_sdk(here))
                    children[:] = []
                    continue
            # Check symlink aliases once without descending through them.
            for child in children:
                alias = here / child
                if alias.is_symlink():
                    with contextlib.suppress(SetupError, OSError):
                        found.add(validate_sdk(alias))
            children[:] = sorted(
                child for child in children
                if depth < max_depth and child not in SKIP_DIRS and
                not child.startswith(('_build', 'build_robot_', 'install_robot_', 'log_robot_'))
                and not child.startswith('.')
            )
    return sorted(found, key=str)


def choose_sdk(candidates, ask=input):
    if candidates:
        print('找到以下机械臂MPSDK（不会选择相机SDK）：')
        for index, root in enumerate(candidates, 1):
            print(f'  {index}. {root}')
        print('  0. 手动输入其它路径')
        while True:
            value = ask('选择SDK编号 [1]: ').strip() or '1'
            if value == '0':
                break
            if value.isdigit() and 1 <= int(value) <= len(candidates):
                return validate_sdk(candidates[int(value) - 1])
            print('请输入列表中的编号。')
    else:
        print('自动搜索未找到完整SDK，请指定解压后的根目录。')
    while True:
        try:
            return validate_sdk(ask('MPSDK根目录: ').strip())
        except (SetupError, OSError) as error:
            print(str(error))


def check_dependencies(root):
    """Trace trusted vendor-library dependencies without starting SDK commands."""
    if not shutil.which('ldd'):
        raise SetupError('未找到ldd，请先安装系统动态库检查工具。')
    env = os.environ.copy()
    env.pop('HIKROBOT_PROGRAMMER_PASSWORD', None)
    env['LD_LIBRARY_PATH'] = str(root / LIB_REL) + (
        ':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    env['LC_ALL'] = 'C'
    try:
        result = subprocess.run(
            ['ldd', '-r', str(root / LIB_REL / 'libMPSDK_PUBLIC_COMPAT.so')],
            env=env, capture_output=True, text=True, timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired):
        raise SetupError('SDK依赖检查无法完成，请按DEPLOYMENT.md手动检查ldd -r。') from None
    if (result.returncode or 'not found' in result.stdout or
            'undefined symbol:' in result.stdout + result.stderr):
        raise SetupError('SDK存在缺失依赖或未解析符号，未写入配置；请按部署文档检查完整SDK库目录。')


def validate_password(password):
    if not password or any(c in password for c in ('\x00', '\n', '\r')):
        raise SetupError('密码不能为空，也不能包含NUL或换行。')
    try:
        length = len(password.encode('utf-8'))
    except UnicodeError:
        raise SetupError('密码必须能以UTF-8编码。') from None
    if length >= 64:
        raise SetupError('密码的UTF-8长度必须小于64字节，与Driver的SDK限制一致。')
    return password


def read_password():
    while True:
        password = getpass.getpass('PROGRAMMER密码（隐藏输入）: ')
        try:
            validate_password(password)
        except SetupError as error:
            print(str(error))
            continue
        if password != getpass.getpass('再次输入密码: '):
            print('两次输入不一致，请重试。')
            continue
        return password


def render_block(root, workspace, ros_setup, password):
    validate_password(password)
    root, workspace, ros_setup = map(clean_path, (root, workspace, ros_setup))
    # Single-quote arbitrary user values; never interpolate them into shell code.
    return f'''{BEGIN}
# Managed by tools/quick_deploy.py. Password is plaintext; keep this file private.
_hikrobot_arm_setup_env() {{
  export HIKROBOT_WS={shlex.quote(str(workspace))}
  if [ -r {shlex.quote(str(ros_setup))} ]; then
    source {shlex.quote(str(ros_setup))}
  fi
  if [ -r "$HIKROBOT_WS/install/setup.bash" ]; then
    source "$HIKROBOT_WS/install/setup.bash"
  fi
  export MPSDK_ROOT={shlex.quote(str(root))}
  export MPSDK_INC="$MPSDK_ROOT/Include/Public"
  export MPSDK_LIB="$MPSDK_ROOT/Libs/Linux/LX64/64"
  export HIKROBOT_MPSDK_ROOT="$MPSDK_ROOT"
  export HIKROBOT_MPSDK_LIBDIR="$MPSDK_LIB"
  export HIKROBOT_PROGRAMMER_PASSWORD={shlex.quote(password)}
  local _hikrobot_ld="$MPSDK_LIB" _hikrobot_entry
  local -a _hikrobot_entries=()
  IFS=: read -r -a _hikrobot_entries <<< "${{LD_LIBRARY_PATH-}}"
  for _hikrobot_entry in "${{_hikrobot_entries[@]}}"; do
    if [[ -n "$_hikrobot_entry" && "$_hikrobot_entry" != "$MPSDK_LIB" ]]; then
      _hikrobot_ld+=":$_hikrobot_entry"
    fi
  done
  export LD_LIBRARY_PATH="$_hikrobot_ld"
}}
_hikrobot_arm_setup_env
unset -f _hikrobot_arm_setup_env
{END}
'''


def merge_block(original, block):
    starts = list(re.finditer(r'^' + re.escape(BEGIN) + r'\r?$', original, re.M))
    ends = list(re.finditer(r'^' + re.escape(END) + r'\r?$', original, re.M))
    if not starts and not ends:
        separator = '' if not original else ('\n' if original.endswith('\n') else '\n\n')
        return original + separator + block
    if len(starts) != 1 or len(ends) != 1 or starts[0].start() >= ends[0].start():
        raise SetupError('bashrc中的部署区块标记不完整或重复，拒绝自动覆盖。')
    end = ends[0].end()
    if original[end:end + 1] == '\n':
        end += 1
    return original[:starts[0].start()] + block + original[end:]


def write_bashrc(path, block):
    """Lock, validate, private backup, atomic replace. Never source the real file."""
    target = clean_path(path)  # Preserve a dotfile symlink by updating its target.
    if not target.parent.is_dir():
        raise SetupError('bashrc父目录不存在。')
    lock_path = target.with_name(target.name + '.hikrobot.lock')
    lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT, 0o600)
    with os.fdopen(lock_fd, 'rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        existed = target.exists()
        if existed and (not target.is_file() or target.stat().st_uid != os.getuid()):
            raise SetupError('只允许修改当前用户拥有的普通bashrc文件。')
        original = target.read_bytes() if existed else b''
        content = merge_block(original.decode('utf-8', errors='surrogateescape'), block)
        payload = content.encode('utf-8', errors='surrogateescape')
        if payload == original:
            target.chmod(0o600)
            return None, False
        fd, temporary = tempfile.mkstemp(prefix='.' + target.name + '.hikrobot-', dir=target.parent)
        backup = None
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            # Syntax check only: existing bashrc commands are never executed here.
            syntax = subprocess.run(['bash', '-n', temporary], capture_output=True, check=False)
            if syntax.returncode:
                # Bash can quote source lines in errors. Never print that output.
                raise SetupError('合并后的bashrc语法检查失败，原文件未修改。')
            if existed:
                backup_fd, backup_name = tempfile.mkstemp(
                    prefix=target.name + '.hikrobot-', suffix='.bak', dir=target.parent)
                backup = Path(backup_name)
                with os.fdopen(backup_fd, 'wb') as stream:
                    stream.write(original)
                    stream.flush()
                    os.fsync(stream.fileno())
            os.replace(temporary, target)  # mkstemp creates mode 0600.
        finally:
            with contextlib.suppress(FileNotFoundError):
                os.unlink(temporary)
        return backup, True


def main(argv=None):
    parser = argparse.ArgumentParser(description='交互保存机械臂SDK、ROS环境和PROGRAMMER密码到bashrc；不连接机器人。')
    parser.add_argument('--sdk-root', help='直接指定SDK根目录，仍会校验')
    parser.add_argument('--search-root', action='append', default=[], help='增加SDK搜索目录，可重复传入；最多向下搜索6层')
    parser.add_argument('--bashrc', default=str(Path.home() / '.bashrc'), help='目标bashrc，默认当前用户~/.bashrc')
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parent.parent))
    parser.add_argument('--ros-setup', default='/opt/ros/humble/setup.bash')
    parser.add_argument('--dry-run', action='store_true', help='检查并显示目标路径，不写bashrc或备份，不显示密码')
    args = parser.parse_args(argv)
    if os.geteuid() == 0:
        raise SetupError('请以普通用户运行，不要sudo，以免配置到root的bashrc。')
    if platform.machine().lower() not in ('x86_64', 'amd64'):
        raise SetupError('当前机械臂SDK shim只支持Linux x86_64布局。')
    if not sys.stdin.isatty() or not sys.stderr.isatty():
        raise SetupError('请在交互终端运行，以保证密码隐藏输入；不接受管道传密码。')
    workspace = clean_path(args.workspace)
    if not (workspace / 'src/hikrobot_mpsdk_vendor/CMakeLists.txt').is_file():
        raise SetupError('工作空间中未找到机械臂vendor包，请检查--workspace。')
    target = clean_path(args.bashrc)
    ros_setup = clean_path(args.ros_setup)
    if args.sdk_root:
        root = validate_sdk(args.sdk_root)
    else:
        roots = [*args.search_root, Path.home(), '/opt', '/usr/local', '/mnt', '/media']
        hints = [os.environ.get('HIKROBOT_MPSDK_ROOT'), os.environ.get('MPSDK_ROOT')]
        root = choose_sdk(discover_sdks(roots, hints))
    check_dependencies(root)
    print('SDK头文件、架构与动态库依赖检查通过。')
    print('SDK:', root)
    print('工作空间:', workspace)
    print('bashrc:', target)
    if not ros_setup.is_file():
        print('提示：ROS setup尚不存在，配置会保留条件加载；仍需按部署指南安装ROS。')
    print('密码将以明文写入bashrc，文件和备份权限为0600；不会显示密码。')
    # Disable getpass's fallback that would echo input if its TTY setup fails.
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('error', getpass.GetPassWarning)
        try:
            password = read_password()
        except getpass.GetPassWarning:
            raise SetupError('无法安全关闭终端回显，未保存密码或配置。') from None
    block = render_block(root, workspace, ros_setup, password)
    if args.dry_run:
        print('预览完成：未写入任何配置、备份或密码。')
        return 0
    if input('写入以上bashrc并保存密码？[Y/n]: ').strip().lower() not in ('', 'y', 'yes'):
        print('已取消，未修改bashrc。')
        return 0
    backup, changed = write_bashrc(target, block)
    print('配置已更新。' if changed else '配置已一致，无需重复写入。')
    if backup:
        print('原文件备份:', backup)
    print('当前父终端不会被子进程自动修改。请执行：')
    print('  source ' + shlex.quote(str(target)))
    print('之后新开的交互式Bash会自动加载。接下来按顺序完成：')
    print('1. 若ROS/系统依赖尚未安装，先按DEPLOYMENT.md第1节安装。')
    print('2. 构建机械臂5包（SDK变量已保存，不需要再输入密码）：')
    print('  cd "$HIKROBOT_WS"')
    print('  colcon build --packages-up-to hikrobot_bringup --executor sequential '
          '--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON '
          '"-DHIKROBOT_MPSDK_ROOT=$HIKROBOT_MPSDK_ROOT"')
    print('  source "$HIKROBOT_WS/install/setup.bash"')
    print('3. 按部署文档检查core/Driver的ldd、ldd -r，再运行离线测试：')
    print('  colcon test --packages-select hikrobot_mpsdk_core hikrobot_driver')
    print('  colcon test-result --verbose')
    print('  python3 -m unittest discover -s tests -p test_quick_deploy.py -v')
    print('4. 复制并审核本机YAML：IP/port、Tool、步长保护和PathID路径。')
    print('5. 仅现场确认后，按文档手动启动Driver、Configure、Activate；Activate会上电。')
    print('完整命令见DEPLOYMENT.md的“配置完成后：首次部署到日常启动”。')
    print('上述仅为下一步提示，本工具未安装依赖、构建包、启动Driver或连接控制器。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except SetupError as error:
        print('部署未完成：' + str(error), file=sys.stderr)
        sys.exit(1)
    except (KeyboardInterrupt, EOFError):
        print('\n已取消，未继续写入配置。', file=sys.stderr)
        sys.exit(130)
    except OSError as error:
        # Do not dump tracebacks, source text or environment values.
        print('部署未完成：文件或系统操作失败（' + (error.strerror or '未知错误') + '）。', file=sys.stderr)
        sys.exit(1)
