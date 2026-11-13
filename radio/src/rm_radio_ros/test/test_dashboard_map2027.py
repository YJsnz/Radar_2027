"""Run the actual map functions in Node, without opening ROS or hardware."""
from pathlib import Path
import subprocess


def test_dashboard_map_coordinates_binding_and_visible_sources():
    root = Path(__file__).resolve().parents[4]
    script = r"""
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const root = process.argv[1];
const source = fs.readFileSync(root + '/radio/apps/match_rx/web/app.js', 'utf8');
const profile = JSON.parse(fs.readFileSync(root + '/radio/apps/match_rx/web/assets/map2027/profile.json'));
const presets = JSON.parse(fs.readFileSync(root + '/vision/calibration_presets_2027.json')).presets;
const nodes = {
  radarMapImage: { src: '', getAttribute() { return this.src; }, setAttribute(k, v) { this[k] = v; } },
  radarMapAssetStatus: {},
  radarMapCanvas: { width: 1500, height: 2800, getContext: () => new Proxy({}, {get: () => () => {}}) },
  showVision: {checked: true}, showRadio: {checked: true},
  showFinal: {checked: true}, showLabels: {checked: false},
};
const context = vm.createContext({console, nodes, profile, presets, assert, process});
vm.runInContext(`
  const $ = name => nodes[name];
  const get = (obj, path, fallback) => path.reduce((v, k) => v?.[k], obj) ?? fallback;
  const RADAR_MAP_ASSET_ROOT = '/assets/map2027/';
  let radarMapProfile = null, radarMapProfileError = '', radarMapOwnSide = 'red';
  let radarLatest = null, radarDirty = false, activePage = 'radar', selectedRobot = null;
  const state = {latest: {}};
  const radarHitTargets = [], radarDisplayPositions = {};
  function fitRadarMapStage() {}
  let fetchedProfile = profile;
  async function fetch() {return {ok:true, json:async () => fetchedProfile};}
` + source.slice(source.indexOf('const radarRoleIds ='), source.indexOf('function coordText('))
  + source.slice(source.indexOf('function drawRadarMap('), source.indexOf('const RADAR_MAP_ASPECT')), context);
vm.runInContext(`
(async () => {
  await loadRadarMapProfile();
  assert.equal(radarMapProfileError, '');
  for (const side of ['R', 'B']) {
    updateRadarMapOrientation(side === 'R' ? 'red' : 'blue');
    const binding = presets.find(p => p.side === side).map_binding;
    assert.equal(radarVisionMapMatches(binding), true);
    assert.ok(nodes.radarMapImage.src.includes(profile.sides[side].image.file));
    assert.equal(radarVisionMapMatches({...binding, side: side === 'R' ? 'B' : 'R'}), false);
    assert.equal(radarVisionMapMatches({...binding, mask_sha256: '0'.repeat(64)}), false);
    assert.equal(radarVisionMapMatches(null), false);
    const points = [{x:0,y:0}, {x:2800,y:1500}, {x:1400,y:750}, {x:800,y:600}];
    const expected = side === 'R' ? [[1500,2800],[0,0],[750,1400],[900,2000]]
                                  : [[0,0],[1500,2800],[750,1400],[600,800]];
    assert.deepEqual(points.map(p => Object.values(radarCanvasPoint(p))), expected);
  }
  updateRadarMapOrientation('red');
  const binding = presets.find(p => p.side === 'R').map_binding;
  const visual = {position_cm:{x:800,y:600}};
  radarLatest = {radar:{data:{side:'red',vision:{map_binding:binding},robots:{
    B1:{visual,final:{...visual,source:'vision'}},
    B3:{radio:{position_cm:{x:1000,y:700},fresh:true},
        final:{position_cm:{x:1000,y:700},source:'radio'}}
  }}}};
  drawRadarMap();
  assert.ok(radarHitTargets.some(t => t.name === 'B1' && t.layer === 'final'));
  for (const invalid of [null, {...binding, points_sha256:'0'.repeat(64)}]) {
    radarLatest.radar.data.vision.map_binding = invalid;
    drawRadarMap();
    assert.equal(radarHitTargets.some(t => t.name === 'B1'), false);
    assert.ok(radarHitTargets.some(t => t.name === 'B3' && t.layer === 'final'));
    updateRadarMapAssetStatus(radarLatest.radar.data.vision);
    assert.ok(nodes.radarMapAssetStatus.textContent.includes(invalid ? '不匹配' : '尚无配套'));
  }
  fetchedProfile = {...profile, coordinate_system:'wrong-domain'};
  await loadRadarMapProfile();
  assert.ok(radarMapProfileError.includes('加载失败'));
  drawRadarMap();
  assert.equal(radarHitTargets.length, 0);
})().catch(error => {console.error(error); process.exitCode = 1;});
`, context);
"""
    result = subprocess.run(['node', '-e', script, str(root)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
