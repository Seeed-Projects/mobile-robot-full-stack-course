#!/usr/bin/env python3
"""Inspect BEVDet ONNX: tensor I/O, plugin nodes, opset, size."""
import sys
import onnx

p = sys.argv[1]
m = onnx.load(p, load_external_data=False)
print("opset:", [x.version for x in m.opset_import if x.domain in ("", "ai.onnx")], "| ir:", m.ir_version)
print("producer:", m.producer_name, m.producer_version)
print("\n== INPUTS ==")
for i in m.graph.input:
    dims = []
    for d in i.type.tensor_type.shape.dim:
        dims.append(d.dim_value if d.HasField("dim_value") else d.dim_param or "?")
    print(f"{i.name:16s} dims={dims}")
print("\n== OUTPUTS ==")
for o in m.graph.output:
    dims = []
    for d in o.type.tensor_type.shape.dim:
        dims.append(d.dim_value if d.HasField("dim_value") else d.dim_param or "?")
    print(f"{o.name:16s} dims={dims}")
print("\n== PLUGIN / CUSTOM NODES ==")
seen = {}
for n in m.graph.node:
    if n.domain not in ("", "ai.onnx"):
        key = (n.domain, n.op_type)
        attrs = {a.name: onnx.helper.get_attribute_value(a) for a in n.attribute}
        seen.setdefault(key, []).append((n.name, attrs))
        print(f"{n.domain}::{n.op_type} '{n.name}' attrs={attrs}")
print("\n== NODE TYPE COUNTS ==")
from collections import Counter
c = Counter((n.domain, n.op_type) for n in m.graph.node)
for k, v in sorted(c.items()):
    if v > 3 or k[0] != "":
        print(f"{k[0]}::{k[1]} x{v}")
    elif k[0] == "":
        print(f"  onnx::{k[1]} x{v}")