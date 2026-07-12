// 常驻 3D 场景(M2,13 §5):controller 级——散装布局(无 machine 段):各 robot 摆地面,
// 按 robot_index 排序网格排布(间距可配,默认 2m);臂用机器人级整机 URDF(自带 EE),
// 被绑 EE 不再单独摆(同 cid 有 assembled 臂 ⇒ 隐藏该 cid 的 ee 节点;精确 ee↔arm 映射 TODO)。
// 关节驱动:全 kind joint_state 聚合(SceneRobot.q × joint_names);EE 关节以 ee_ 前缀写进
// 臂的整机模型(mimic 从动由 urdf-loader 0.13 原生联动)。无 URDF 的 robot(如 base)画占位盒。
// 选中高亮(其余 ghost/隐藏切换与 3D 点击选中 = M3)。

import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { api } from "../api";
import type { SceneRobot } from "../types";

interface Props {
  robots: SceneRobot[];       // ee_scene 轮询(~30Hz)
  selected: string | null;    // 选中 robot prefix(高亮)
  spacing: number;            // 散装网格间距 m
  height?: number;
}

type Slot = {
  group: THREE.Group;               // 网格位安放点
  robot: URDFRobot | null;          // 已加载的 URDF 模型(null=占位盒/加载中)
  assembled: boolean;               // 臂:整机(含 EE)
  kind: string;
  placeholder: THREE.Mesh | null;
  loading: boolean;
  lastFetch: number;                // 上次 URDF 拉取时刻(ms;臂未拼装时周期重拉)
  highlighted: boolean;
};

const HIGHLIGHT = new THREE.Color(0x2a6fbb);

export function MachineViewer({ robots, selected, spacing, height = 340 }: Props) {
  const mountRef = useRef<HTMLDivElement>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const slotsRef = useRef<Map<string, Slot>>(new Map());
  const worldRef = useRef<THREE.Group | null>(null);
  const propsRef = useRef({ robots, selected, spacing });
  propsRef.current = { robots, selected, spacing };

  useEffect(() => {
    const mount = mountRef.current!;
    const W = mount.clientWidth || 800;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1d23);
    const camera = new THREE.PerspectiveCamera(50, W / height, 0.01, 200);
    camera.position.set(2.2, -2.6, 1.8);
    camera.up.set(0, 0, 1); // URDF Z-up
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(W, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    mount.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.2);
    controlsRef.current = controls;

    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(2, 2, 4);
    scene.add(dir);
    const grid = new THREE.GridHelper(12, 60, 0x444444, 0x2a2a2a).rotateX(Math.PI / 2);
    (grid.material as THREE.Material).transparent = true;
    (grid.material as THREE.Material).opacity = 0.3;
    scene.add(grid);

    const world = new THREE.Group();
    scene.add(world);
    worldRef.current = world;

    // 每帧:布局 + 关节驱动 + 高亮(数据从 propsRef 拉,避免 effect 重建 three 场景)
    let raf = 0;
    const animate = () => {
      applyFrame();
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(animate);
    };
    animate();
    const onResize = () => {
      const w = mount.clientWidth || 800;
      camera.aspect = w / height; camera.updateProjectionMatrix(); renderer.setSize(w, height);
    };
    window.addEventListener("resize", onResize);
    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener("resize", onResize);
      renderer.dispose();
      if (renderer.domElement.parentNode === mount) mount.removeChild(renderer.domElement);
      slotsRef.current.forEach((s) => disposeSlot(s));
      slotsRef.current.clear();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [height]);

  function disposeSlot(s: Slot) {
    worldRef.current?.remove(s.group);
    s.group.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh) {
        m.geometry?.dispose();
        const mat = m.material;
        if (Array.isArray(mat)) mat.forEach((x) => x.dispose()); else (mat as THREE.Material)?.dispose();
      }
    });
  }

  function disposeRobot(slot: Slot) {
    if (!slot.robot) return;
    slot.group.remove(slot.robot);
    slot.robot.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.isMesh) {
        m.geometry?.dispose();
        const mat = m.material;
        if (Array.isArray(mat)) mat.forEach((x) => x.dispose()); else (mat as THREE.Material)?.dispose();
      }
    });
    slot.robot = null;
  }

  function loadUrdfInto(slot: Slot, prefix: string, kindName: string) {
    slot.loading = true;
    slot.lastFetch = performance.now();
    api.consoleGetUrdf(prefix, kindName).then((u) => {
      slot.loading = false;
      if (!u || !u.xml) return; // 无 URDF:保留占位盒
      if (slot.robot && slot.assembled === u.assembled) return; // 已有同形态模型,不重建
      const loader = new URDFLoader();
      loader.packages = { xpkg_urdf_firefly_y6: "/urdf", hex_gp80_description: "/urdf/gp80", hex_gr80_description: "/urdf/gr80" };
      (loader as any).loadMeshCb = (
        url: string, manager: THREE.LoadingManager, _material: THREE.Material,
        onComplete: (obj: THREE.Object3D | null, err?: Error) => void,
      ) => {
        new STLLoader(manager).load(
          url,
          (geom) => onComplete(new THREE.Mesh(geom, new THREE.MeshPhongMaterial({ color: 0xbfc4cc }))),
          undefined,
          (err) => onComplete(null, err as Error),
        );
      };
      try {
        const robot = loader.parse(u.xml);
        disposeRobot(slot); // 拼装形态升级(臂-only → 整机):替换旧模型
        slot.robot = robot;
        slot.highlighted = false; // 新模型重新走高亮着色
        slot.assembled = u.assembled;
        if (slot.placeholder) { slot.group.remove(slot.placeholder); slot.placeholder = null; }
        slot.group.add(robot);
      } catch (e) {
        console.warn("URDF parse failed", prefix, e);
      }
    }).catch(() => { slot.loading = false; });
  }

  function applyFrame() {
    const world = worldRef.current;
    if (!world) return;
    const { robots, selected, spacing } = propsRef.current;
    const slots = slotsRef.current;

    // 被绑 EE 隐藏:同 cid 存在 assembled 臂 ⇒ 该 cid 的 ee 不单独摆(13 §1;精确映射 TODO)。
    const assembledCids = new Set<string>();
    slots.forEach((s, prefix) => {
      if (s.assembled) {
        const r = robots.find((x) => x.prefix === prefix);
        if (r) assembledCids.add(r.cid);
      }
    });
    const visible = robots.filter((r) => !(r.kind_name === "ee" && assembledCids.has(r.cid)));

    // 建/删 slot
    const seen = new Set<string>();
    visible.forEach((r) => {
      seen.add(r.prefix);
      if (!slots.has(r.prefix)) {
        const group = new THREE.Group();
        // 占位盒(base 等无 URDF,或加载中):40cm 立方线框
        const box = new THREE.Mesh(
          new THREE.BoxGeometry(0.4, 0.4, 0.25),
          new THREE.MeshPhongMaterial({ color: 0x555c66, transparent: true, opacity: 0.6 }),
        );
        box.position.z = 0.125;
        group.add(box);
        world.add(group);
        const slot: Slot = { group, robot: null, assembled: false, kind: r.kind_name, placeholder: box, loading: false, lastFetch: 0, highlighted: false };
        slots.set(r.prefix, slot);
        loadUrdfInto(slot, r.prefix, r.kind_name);
      }
    });
    slots.forEach((s, prefix) => {
      const inScene = seen.has(prefix);
      s.group.visible = inScene;      // 被绑 EE / 消失的 robot:隐藏但保留(再现时秒回)
    });

    // 散装网格布局(按 visible 顺序 = 后端已按 cid+robot_index 排序)
    const n = visible.length;
    const cols = Math.max(1, Math.ceil(Math.sqrt(n)));
    visible.forEach((r, i) => {
      const s = slots.get(r.prefix)!;
      const col = i % cols, row = Math.floor(i / cols);
      s.group.position.set(col * spacing - ((cols - 1) * spacing) / 2, -row * spacing, 0);
    });

    // 关节驱动 + EE 关节写进臂的整机模型(ee_ 前缀;mimic 由 urdf-loader 联动)
    const byCid = new Map<string, SceneRobot[]>();
    robots.forEach((r) => { const a = byCid.get(r.cid) ?? []; a.push(r); byCid.set(r.cid, a); });
    visible.forEach((r) => {
      const s = slots.get(r.prefix);
      if (!s?.robot) return;
      r.joint_names.forEach((name, i) => {
        if (r.q[i] != null && s.robot!.joints[name]) s.robot!.setJointValue(name, r.q[i]);
      });
      if (r.kind_name === "arm" && s.assembled) {
        (byCid.get(r.cid) ?? []).filter((x) => x.kind_name === "ee").forEach((ee) => {
          ee.joint_names.forEach((name, i) => {
            const jn = `ee_${name}`;
            if (ee.q[i] != null && s.robot!.joints[jn]) s.robot!.setJointValue(jn, ee.q[i]);
          });
        });
      }
    });

    // 臂未拼装(EE 拼装比 GUI 首查晚就绪)→ 每 5s 重拉 URDF,拼好即替换成整机模型
    const now = performance.now();
    visible.forEach((r) => {
      const s = slots.get(r.prefix);
      if (s && s.kind === "arm" && !s.assembled && !s.loading && now - s.lastFetch > 5000) {
        loadUrdfInto(s, r.prefix, r.kind_name);
      }
    });

    // 视角跟随:orbit 目标平滑趋向选中 robot(未选中 → 场景原点)
    const controls = controlsRef.current;
    if (controls) {
      const tgt = new THREE.Vector3(0, 0, 0.2);
      if (selected) {
        const s = slots.get(selected);
        if (s) { s.group.getWorldPosition(tgt); tgt.z += 0.25; }
      }
      controls.target.lerp(tgt, 0.08); // 平滑跟随,不瞬跳
    }

    // 选中高亮(emissive 着色;ghost/隐藏切换 = M3)
    slots.forEach((s, prefix) => {
      const want = prefix === selected;
      if (want === s.highlighted) return;
      s.highlighted = want;
      s.group.traverse((o) => {
        const m = o as THREE.Mesh;
        if (m.isMesh && !Array.isArray(m.material)) {
          const mat = m.material as THREE.MeshPhongMaterial;
          if (mat.emissive) mat.emissive.set(want ? HIGHLIGHT : 0x000000);
        }
      });
    });
  }

  return <div ref={mountRef} style={{ width: "100%", height, borderRadius: 8, overflow: "hidden" }} />;
}
