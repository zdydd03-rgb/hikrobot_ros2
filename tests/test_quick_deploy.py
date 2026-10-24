"""Offline configuration tests. Never read or write the real user's bashrc."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import stat
import struct
import subprocess
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location(
    'quick_deploy', Path(__file__).resolve().parents[1] / 'tools/quick_deploy.py')
deploy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(deploy)


class QuickDeployTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='hikrobot-quick-deploy-test-')
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name)
        self.sdk = self.make_sdk(self.base / "SDK with spaces ' and $literal")
        self.workspace = self.base / 'workspace'
        vendor = self.workspace / 'src/hikrobot_mpsdk_vendor/CMakeLists.txt'
        vendor.parent.mkdir(parents=True)
        vendor.write_text('# test fixture\n')
        self.ros = self.base / 'missing-ros-setup.bash'
        self.bashrc = self.base / 'bashrc'
        self.password = "test-only ' $value `unused` ! # 密码"

    @staticmethod
    def make_sdk(root):
        include = root / 'Include/Public'
        include.mkdir(parents=True)
        for name in deploy.HEADERS:
            (include / name).write_text('// fixture\n')
        library = root / deploy.LIB_REL / 'libMPSDK_PUBLIC_COMPAT.so'
        library.parent.mkdir(parents=True)
        header = bytearray(20)
        header[:6] = b'\x7fELF\x02\x01'
        struct.pack_into('<HH', header, 16, 3, 62)
        library.write_bytes(header)
        return root

    def block(self, password=None):
        return deploy.render_block(self.sdk, self.workspace, self.ros,
                                   self.password if password is None else password)

    def test_validates_complete_sdk_and_rejects_missing_headers(self):
        self.assertEqual(deploy.validate_sdk(self.sdk), self.sdk)
        (self.sdk / 'Include/Public/MP_IO_API.h').unlink()
        with self.assertRaises(deploy.SetupError):
            deploy.validate_sdk(self.sdk)

    def test_rejects_non_elf_and_wrong_architecture(self):
        library = self.sdk / deploy.LIB_REL / 'libMPSDK_PUBLIC_COMPAT.so'
        for content in (b'not an ELF library', b'\x7fELF\x01\x01' + bytes(14)):
            library.write_bytes(content)
            with self.assertRaises(deploy.SetupError):
                deploy.validate_sdk(self.sdk)

    def test_discovers_deduplicates_aliases_and_prunes_build(self):
        alias = self.base / 'alias'
        alias.symlink_to(self.sdk, target_is_directory=True)
        self.make_sdk(self.base / 'build/ignored-sdk')
        found = deploy.discover_sdks([self.base, self.base], [str(alias), str(self.sdk)])
        self.assertEqual(found, [self.sdk])

    def test_search_depth_and_manual_selection(self):
        self.assertEqual(deploy.discover_sdks([self.base], max_depth=0), [])
        with contextlib.redirect_stdout(io.StringIO()):
            selected = deploy.choose_sdk([self.sdk], ask=mock.Mock(return_value=''))
        self.assertEqual(selected, self.sdk)

    def test_password_matches_sdk_utf8_limit(self):
        self.assertEqual(deploy.validate_password('a' * 63), 'a' * 63)
        for value in ('', 'a' * 64, '密' * 22, 'a\nb', 'a\x00b', 'a\rb'):
            with self.assertRaises(deploy.SetupError):
                deploy.validate_password(value)

    def test_append_preserves_original_and_update_is_idempotent(self):
        original = '# user config\nexport OTHER=unchanged'  # no final newline
        first = deploy.merge_block(original, self.block())
        self.assertTrue(first.startswith(original + '\n\n'))
        self.assertEqual(deploy.merge_block(first, self.block()), first)
        changed = deploy.merge_block(first + '# trailing user note\n', self.block('updated-test'))
        self.assertEqual(changed.count(deploy.BEGIN), 1)
        self.assertTrue(changed.endswith('# trailing user note\n'))
        self.assertIn('updated-test', changed)

    def test_bad_markers_fail_without_touching_original(self):
        for content in (deploy.BEGIN + '\n', deploy.END + '\n',
                        deploy.END + '\n' + deploy.BEGIN + '\n',
                        self.block() + self.block()):
            self.bashrc.write_text(content)
            with self.assertRaises(deploy.SetupError):
                deploy.write_bashrc(self.bashrc, self.block())
            self.assertEqual(self.bashrc.read_text(), content)
        self.assertEqual(list(self.base.glob('bashrc.hikrobot-*.bak')), [])

    def test_private_backup_and_no_duplicate_backup_for_unchanged_config(self):
        original = b'# user config\nexport OTHER=unchanged\n'
        self.bashrc.write_bytes(original)
        self.bashrc.chmod(0o644)
        backup, changed = deploy.write_bashrc(self.bashrc, self.block())
        self.assertTrue(changed)
        self.assertEqual(backup.read_bytes(), original)
        for path in (self.bashrc, backup):
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
        self.assertEqual(deploy.write_bashrc(self.bashrc, self.block()), (None, False))
        self.assertEqual(len(list(self.base.glob('bashrc.hikrobot-*.bak'))), 1)

    def test_dotfile_symlink_is_preserved(self):
        actual = self.base / 'dotfile-target'
        actual.write_text('# original\n')
        self.bashrc.symlink_to(actual)
        deploy.write_bashrc(self.bashrc, self.block())
        self.assertTrue(self.bashrc.is_symlink())
        self.assertIn(deploy.BEGIN, actual.read_text())

    def test_syntax_error_preserves_original_and_does_not_echo_secret(self):
        original = '# synthetic invalid file\nif then\n'
        self.bashrc.write_text(original)
        with self.assertRaises(deploy.SetupError) as error:
            deploy.write_bashrc(self.bashrc, self.block())
        self.assertNotIn(self.password, str(error.exception))
        self.assertEqual(self.bashrc.read_text(), original)
        self.assertEqual(list(self.base.glob('bashrc.hikrobot-*.bak')), [])

    def test_failed_replace_preserves_original(self):
        self.bashrc.write_text('# original\n')
        with mock.patch.object(deploy.os, 'replace', side_effect=OSError('fixture failure')):
            with self.assertRaises(OSError):
                deploy.write_bashrc(self.bashrc, self.block())
        self.assertEqual(self.bashrc.read_text(), '# original\n')
        self.assertEqual(list(self.base.glob('.bashrc.hikrobot-*')), [])

    def test_shell_quotes_password_and_paths_without_executing_substitutions(self):
        marker = self.base / 'MUST_NOT_EXIST'
        password = "test'$(touch MUST_NOT_EXIST) `true` !"
        deploy.write_bashrc(self.bashrc, self.block(password))
        env = os.environ.copy()
        sdk_lib = str(self.sdk / deploy.LIB_REL)
        env['LD_LIBRARY_PATH'] = '/some/other/lib:' + sdk_lib + ':' + sdk_lib
        keys = ['MPSDK_ROOT', 'MPSDK_INC', 'MPSDK_LIB', 'HIKROBOT_MPSDK_ROOT',
                'HIKROBOT_MPSDK_LIBDIR', 'HIKROBOT_WS', 'HIKROBOT_PROGRAMMER_PASSWORD',
                'LD_LIBRARY_PATH']
        code = 'import os,json;print(json.dumps({k:os.environ[k] for k in ' + repr(keys) + '}))'
        command = 'source "$1"; source "$1"; python3 -c "$2"'
        result = subprocess.run(['bash', '--noprofile', '--norc', '-c', command,
                                 'fixture', str(self.bashrc), code],
                                env=env, cwd=self.base, capture_output=True, text=True, check=True)
        saved = json.loads(result.stdout)
        self.assertEqual(saved['HIKROBOT_PROGRAMMER_PASSWORD'], password)
        self.assertEqual(saved['MPSDK_ROOT'], str(self.sdk))
        self.assertEqual(saved['MPSDK_INC'], str(self.sdk / 'Include/Public'))
        self.assertEqual(saved['MPSDK_LIB'], sdk_lib)
        self.assertEqual(saved['HIKROBOT_MPSDK_LIBDIR'], sdk_lib)
        self.assertEqual(saved['HIKROBOT_MPSDK_ROOT'], str(self.sdk))
        self.assertEqual(saved['HIKROBOT_WS'], str(self.workspace))
        self.assertEqual(saved['LD_LIBRARY_PATH'].split(':').count(sdk_lib), 1)
        self.assertEqual(saved['LD_LIBRARY_PATH'].split(':')[0], sdk_lib)
        self.assertFalse(marker.exists())

    def test_ros_and_install_sources_are_optional_and_load_in_order(self):
        self.ros.write_text('export QUICK_DEPLOY_FIXTURE=ros\n')
        install = self.workspace / 'install/setup.bash'
        install.parent.mkdir()
        install.write_text('export QUICK_DEPLOY_FIXTURE="$QUICK_DEPLOY_FIXTURE:install"\n')
        deploy.write_bashrc(self.bashrc, self.block())
        result = subprocess.run(['bash', '--noprofile', '--norc', '-c',
                                 'source "$1"; printf "%s" "$QUICK_DEPLOY_FIXTURE"',
                                 'fixture', str(self.bashrc)],
                                capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout, 'ros:install')

    def test_dependency_failures_are_rejected(self):
        for output in ('dependency => not found', 'undefined symbol: fixture'):
            with mock.patch.object(deploy.shutil, 'which', return_value='/usr/bin/ldd'), \
                 mock.patch.object(deploy.subprocess, 'run',
                                   return_value=subprocess.CompletedProcess([], 0, output, '')):
                with self.assertRaises(deploy.SetupError):
                    deploy.check_dependencies(self.sdk)

    def test_main_dry_run_never_saves_or_prints_password(self):
        output = io.StringIO()
        args = ['--sdk-root', str(self.sdk), '--workspace', str(self.workspace),
                '--ros-setup', str(self.ros), '--bashrc', str(self.bashrc), '--dry-run']
        with mock.patch.object(deploy.os, 'geteuid', return_value=1000), \
             mock.patch.object(deploy.platform, 'machine', return_value='x86_64'), \
             mock.patch.object(deploy.sys.stdin, 'isatty', return_value=True), \
             mock.patch.object(deploy.sys.stderr, 'isatty', return_value=True), \
             mock.patch.object(deploy, 'check_dependencies'), \
             mock.patch.object(deploy.getpass, 'getpass', return_value=self.password), \
             contextlib.redirect_stdout(output):
            self.assertEqual(deploy.main(args), 0)
        self.assertNotIn(self.password, output.getvalue())
        self.assertFalse(self.bashrc.exists())
        self.assertFalse(self.bashrc.with_name('bashrc.hikrobot.lock').exists())

    def test_main_confirmed_write_and_cancel_do_not_print_password(self):
        args = ['--sdk-root', str(self.sdk), '--workspace', str(self.workspace),
                '--ros-setup', str(self.ros), '--bashrc', str(self.bashrc)]
        for response, exists in [('n', False), ('y', True)]:
            output = io.StringIO()
            with mock.patch.object(deploy.os, 'geteuid', return_value=1000), \
                 mock.patch.object(deploy.platform, 'machine', return_value='x86_64'), \
                 mock.patch.object(deploy.sys.stdin, 'isatty', return_value=True), \
                 mock.patch.object(deploy.sys.stderr, 'isatty', return_value=True), \
                 mock.patch.object(deploy, 'check_dependencies'), \
                 mock.patch.object(deploy.getpass, 'getpass', return_value=self.password), \
                 mock.patch('builtins.input', return_value=response), \
                 contextlib.redirect_stdout(output):
                self.assertEqual(deploy.main(args), 0)
            self.assertNotIn(self.password, output.getvalue())
            self.assertEqual(self.bashrc.exists(), exists)
        self.assertIn(deploy.BEGIN, self.bashrc.read_text())

    def test_password_mismatch_reprompts_without_printing_values(self):
        output = io.StringIO()
        with mock.patch.object(deploy.getpass, 'getpass',
                               side_effect=['first-test', 'second-test', 'last-test', 'last-test']), \
             contextlib.redirect_stdout(output):
            self.assertEqual(deploy.read_password(), 'last-test')
        for value in ['first-test', 'second-test', 'last-test']:
            self.assertNotIn(value, output.getvalue())

    def test_main_refuses_noninteractive_input_before_any_write(self):
        with mock.patch.object(deploy.os, 'geteuid', return_value=1000), \
             mock.patch.object(deploy.platform, 'machine', return_value='x86_64'), \
             mock.patch.object(deploy.sys.stdin, 'isatty', return_value=False):
            with self.assertRaises(deploy.SetupError):
                deploy.main(['--bashrc', str(self.bashrc)])
        self.assertFalse(self.bashrc.exists())


if __name__ == '__main__':
    unittest.main()
