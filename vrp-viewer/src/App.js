import React, { useEffect, useState, useRef } from "react";
import InteractiveVRPViewer from "./InteractiveVRPViewer";

export default function App() {
  const [caseList, setCaseList] = useState([]);        // [{name, steps}, ...]
  const [selectedCase, setSelectedCase] = useState(null);
  const [stepList, setStepList] = useState([]);        // ["step_0.json", ...]
  const [selectedStep, setSelectedStep] = useState(null);
  const [data, setData] = useState(null);

  // ノード選択（FastVRPViewer から受け取る）
  const [selectedNode, setSelectedNode] = useState(null);

  // ビューア領域のサイズ計測
  const viewerRef = useRef(null);
  const [viewerSize, setViewerSize] = useState({ width: 0, height: 0 });
  useEffect(() => {
    const measure = () => {
      if (!viewerRef.current) return;
      setViewerSize({
        width: viewerRef.current.clientWidth,
        height: viewerRef.current.clientHeight,
      });
    };
    measure();
    window.addEventListener("resize", measure);
    return () => window.removeEventListener("resize", measure);
  }, []);

  // index.json 正規化
  const normalizeIndexJson = (json) => {
    if (json && Array.isArray(json.cases)) {
      return json.cases
        .map((c) => {
          if (typeof c === "string") return { name: c, steps: [] };
          if (typeof c === "object" && c.name) return { name: c.name, steps: c.steps || [] };
          return null;
        })
        .filter(Boolean);
    }
    if (Array.isArray(json)) {
      return json.map((fname) => {
        const base = fname.replace(/\.json$/, "");
        return { name: base, steps: [] };
      });
    }
    return [];
  };

  // 初回: cases 取得
  useEffect(() => {
    fetch(process.env.PUBLIC_URL + "/vrp_data/index.json")
      .then((res) => {
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
      })
      .then((json) => {
        const cases = normalizeIndexJson(json);
        setCaseList(cases);
        if (cases.length > 0) {
          setSelectedCase(cases[0].name);
          if (cases[0].steps && cases[0].steps.length > 0) {
            setStepList(cases[0].steps);
            setSelectedStep(cases[0].steps[0]);
          }
        }
      })
      .catch((err) => {
        console.error("index.json 読み込みエラー:", err);
        setCaseList([]);
      });
  }, []);

  // Case 切替時: steps 取得
  useEffect(() => {
    if (!selectedCase) return;
    const caseObj = caseList.find((c) => c.name === selectedCase);
    if (caseObj && caseObj.steps && caseObj.steps.length > 0) {
      setStepList(caseObj.steps);
      setSelectedStep(caseObj.steps[0]);
      return;
    }
    fetch(process.env.PUBLIC_URL + `/vrp_data/${selectedCase}/index.json`)
      .then((res) => {
        if (!res.ok) throw new Error(`Case index not found: ${res.status}`);
        return res.json();
      })
      .then((json) => {
        if (Array.isArray(json.steps)) {
          setStepList(json.steps);
          setSelectedStep(json.steps[0] || null);
        } else if (Array.isArray(json)) {
          setStepList(json);
          setSelectedStep(json[0] || null);
        } else {
          setStepList([]);
          setSelectedStep(null);
        }
      })
      .catch((err) => {
        console.warn("case 内の index.json 読み込みに失敗:", err);
        setStepList([]);
        setSelectedStep(null);
      });
  }, [selectedCase, caseList]);

  // データ読み込み
  useEffect(() => {
    if (!selectedCase || !selectedStep) return;
    fetch(process.env.PUBLIC_URL + `/vrp_data/${selectedCase}/${selectedStep}`)
      .then((res) => {
        if (!res.ok) throw new Error(`Data not found: ${res.status}`);
        return res.json();
      })
      .then((json) => setData(json))
      .catch((err) => {
        console.error("VRPデータ読み込みエラー:", err);
        setData(null);
      });
  }, [selectedCase, selectedStep]);

  // --- 追加: Case 前後移動ハンドラ（最小変更） ---
  const currentCaseIndex = caseList.findIndex((c) => c.name === selectedCase);
  const hasPrevCase = currentCaseIndex > 0;
  const hasNextCase = currentCaseIndex >= 0 && currentCaseIndex < caseList.length - 1;

  const goPrevCase = () => {
    if (!hasPrevCase) return;
    setSelectedCase(caseList[currentCaseIndex - 1].name);
  };
  const goNextCase = () => {
    if (!hasNextCase) return;
    setSelectedCase(caseList[currentCaseIndex + 1].name);
  };

  // --- 既存: Step 前後移動ハンドラ ---
  const currentStepIndex = stepList.findIndex((s) => s === selectedStep);
  const hasPrev = currentStepIndex > 0;
  const hasNext = currentStepIndex >= 0 && currentStepIndex < stepList.length - 1;

  const goPrevStep = () => {
    if (!hasPrev) return;
    setSelectedStep(stepList[currentStepIndex - 1]);
  };
  const goNextStep = () => {
    if (!hasNext) return;
    setSelectedStep(stepList[currentStepIndex + 1]);
  };

  // --- ノード情報を横並びのチップで表示する部品 ---
  const NodeInfoChips = ({ info }) => {
    const wrapStyle = {
      display: "flex",
      alignItems: "center",
      gap: 8,
      height: 56,
      overflowX: "auto",
      overflowY: "hidden",
      whiteSpace: "nowrap",
    };
    const chipStyle = {
      display: "inline-flex",
      alignItems: "center",
      gap: 6,
      padding: "6px 10px",
      border: "1px solid #ddd",
      borderRadius: 999,
      background: "#fff",
      fontSize: 13,
      maxWidth: 260,
      textOverflow: "ellipsis",
      overflow: "hidden",
    };
    const labelStyle = { opacity: 0.6 };

    if (!info) {
      return (
        <div style={wrapStyle}>
          <span style={{ color: "#666" }}>ノードをクリックしてください。</span>
        </div>
      );
    }
    return (
      <div style={wrapStyle} title={`Node ${info.id}`}>
        <span style={chipStyle}><span style={labelStyle}>ID</span> {info.id}</span>
        <span style={chipStyle}>
          <span style={labelStyle}>座標</span> ({info.x.toFixed(1)}, {info.y.toFixed(1)})
        </span>
        <span style={chipStyle}><span style={labelStyle}>種類</span> {info.kind}</span>
        {info.partnerId && (
          <>
            <span style={chipStyle}><span style={labelStyle}>対応ID</span> {info.partnerId}</span>
            {info.partnerCoord && (
              <span style={chipStyle}>
                <span style={labelStyle}>対応座標</span> (
                {info.partnerCoord.x.toFixed(1)}, {info.partnerCoord.y.toFixed(1)})
              </span>
            )}
          </>
        )}
      </div>
    );
  };

  return (
    <div style={{ minHeight: "100vh", display: "flex", flexDirection: "column", padding: 16 }}>
      {/* ヘッダー：左タイトル／右ノード情報（1行固定） */}
      <div
        style={{
          display: "flex",
          alignItems: "center",
          justifyContent: "space-between",
          gap: 16,
          flexWrap: "nowrap",
        }}
      >
        <h1 style={{ margin: 0, lineHeight: "56px" }}>VRP Route Viewer</h1>

        {/* 右側：ノード情報（横並びチップ／横スクロール） */}
        <div
          style={{
            minWidth: 360,
            maxWidth: "50vw",
            border: "1px solid #ccc",
            borderRadius: 10,
            background: "#f9f9f9",
            padding: "4px 8px",
            height: 56,
            boxSizing: "border-box",
          }}
        >
          <NodeInfoChips info={selectedNode} />
        </div>
      </div>

      {/* セレクタ行 */}
      <div style={{ margin: "12px 0", display: "flex", alignItems: "center", gap: 8 }}>
        <label style={{ marginRight: 8 }}>Case:</label>

        {/* 追加: 前のCaseボタン */}
        <button
          onClick={goPrevCase}
          disabled={!hasPrevCase}
          title="前のケースへ"
          style={{ padding: "4px 8px" }}
        >
          ◀
        </button>

        <select value={selectedCase || ""} onChange={(e) => setSelectedCase(e.target.value)}>
          {caseList.map((c, idx) => (
            <option key={`${c.name}-${idx}`} value={c.name}>{c.name}</option>
          ))}
        </select>

        {/* 追加: 次のCaseボタン */}
        <button
          onClick={goNextCase}
          disabled={!hasNextCase}
          title="次のケースへ"
          style={{ padding: "4px 8px" }}
        >
          ▶
        </button>

        <label style={{ marginLeft: 16, marginRight: 8 }}>Step:</label>

        {/* 追加: 前へボタン（Step） */}
        <button
          onClick={goPrevStep}
          disabled={!hasPrev}
          title="前のステップへ"
          style={{ padding: "4px 8px" }}
        >
          ◀
        </button>

        {/* 既存のプルダウン（Step） */}
        <select value={selectedStep || ""} onChange={(e) => setSelectedStep(e.target.value)}>
          {stepList.map((s, idx) => (
            <option key={`${s}-${idx}`} value={s}>{s}</option>
          ))}
        </select>

        {/* 追加: 次へボタン（Step） */}
        <button
          onClick={goNextStep}
          disabled={!hasNext}
          title="次のステップへ"
          style={{ padding: "4px 8px" }}
        >
          ▶
        </button>
      </div>

      {/* ビューア領域：残り高さすべて */}
      <div ref={viewerRef} style={{ flex: 1, minHeight: 0 }}>
        {data ? (
          <InteractiveVRPViewer
            customers={data.customers}
            routes={data.routes}
            PD_pairs={data.PD_pairs}
            depot_id_list={data.depot_id_list}
            vehicle_num_list={data.vehicle_num_list}
            width={viewerSize.width}
            height={viewerSize.height}
            onSelectNode={setSelectedNode}
          />
        ) : (
          <div>データを読み込み中、またはデータが見つかりません。</div>
        )}
      </div>
    </div>
  );
}
