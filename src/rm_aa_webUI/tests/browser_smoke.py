"""Exercise an isolated --demo workspace using an installed Chromium/Edge browser."""
import argparse
from pathlib import Path
import sys
import httpx
from playwright.sync_api import sync_playwright, expect


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8086')
    parser.add_argument('--browser', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    status = httpx.get(args.url + '/api/status').json()
    assert status['demo'] and 'webui-smoke-' in status['workspace'], 'Use a disposable copied workspace for edits'
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    errors = []
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.browser, headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 980}, accept_downloads=True)
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto(args.url)
        expect(page.get_by_text('demo.target_yaw', exact=True)).to_be_visible()
        card = page.locator('[data-metric="demo.target_yaw"]')
        page.get_by_role('button', name='暂停显示', exact=True).click()
        frozen = card.locator('.chart-value').inner_text()
        page.wait_for_timeout(300)
        assert card.locator('.chart-value').inner_text() == frozen
        with page.expect_download() as event:
            page.get_by_role('button', name='导出全部 CSV', exact=True).click()
        event.value.save_as(output / 'ui-export.csv')
        assert 'demo.target_yaw' in (output / 'ui-export.csv').read_text(encoding='utf-8-sig')
        page.get_by_role('button', name='继续显示', exact=True).click()
        card.get_by_role('button', name='折叠 demo.target_yaw', exact=True).click()
        expect(card.locator('canvas')).to_be_hidden()
        card.get_by_role('button', name='展开 demo.target_yaw', exact=True).click()
        page.screenshot(path=output / 'ui-monitor.png', full_page=True)

        page.get_by_role('button', name='参数配置', exact=True).click()
        yaml_path = 'src/rm_aa_bringup/config/perception.yaml'
        page.get_by_label('配置文件', exact=True).select_option(yaml_path)
        fps = page.get_by_label('perception / ros__parameters / source / fps', exact=True)
        fps.fill('75')
        page.get_by_role('button', name='保存到文件', exact=True).click()
        expect(page.locator('#notice')).to_contain_text('已写入文件')
        saved = httpx.get(args.url + '/api/config', params={'path': yaml_path}).json()
        assert any(field['key'].endswith('/fps') and field['value'] == 75 for field in saved['fields'])
        assert '# camera mode:' in saved['content']
        page.get_by_label('筛选参数').fill('source')
        page.screenshot(path=output / 'ui-config.png', full_page=True)
        page.get_by_label('筛选参数').fill('')

        launch_path = 'src/rm_aa_bringup/launch/perception.launch.py'
        page.get_by_label('配置文件', exact=True).select_option(launch_path)
        page.get_by_label('启动参数 / port', exact=True).fill('8088')
        page.get_by_role('button', name='保存到文件', exact=True).click()
        expect(page.locator('#notice')).to_contain_text('已写入文件')
        saved_launch = httpx.get(args.url + '/api/config', params={'path': launch_path}).json()
        assert "default_value='8088'" in saved_launch['content']
        page.get_by_role('button', name='源文件', exact=True).click()
        editor = page.get_by_label('配置源文件', exact=True)
        editor.fill('def broken(\n')
        page.get_by_role('button', name='保存到文件', exact=True).click()
        expect(page.locator('#notice')).to_have_class('error')
        assert httpx.get(args.url + '/api/config', params={'path': launch_path}).json()['revision'] == saved_launch['revision']
        editor.fill(saved_launch['content'] + '\n# browser raw edit\n')
        page.get_by_role('button', name='保存到文件', exact=True).click()
        expect(page.locator('#notice')).to_contain_text('已写入文件')

        page.get_by_role('button', name='实时监控', exact=True).click()
        page.get_by_role('button', name='暂停显示', exact=True).click()
        page.get_by_role('button', name='清空曲线', exact=True).click()
        expect(page.locator('#metricCount')).to_have_text('0 个指标')
        page.get_by_role('button', name='继续显示', exact=True).click()
        expect(page.get_by_text('demo.target_yaw', exact=True)).to_be_visible()
        page.set_viewport_size({'width': 390, 'height': 844})
        page.screenshot(path=output / 'ui-mobile.png', full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
        assert not errors, errors
        browser.close()
    print('Browser PASS: curves, pause, CSV, collapse, YAML/launch save, invalid syntax, raw editor, clear, mobile layout')


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
