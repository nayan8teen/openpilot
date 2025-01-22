import os, sys, pickle, time
import numpy as np
if "FLOAT16" not in os.environ: os.environ["FLOAT16"] = "1"
if "IMAGE" not in os.environ: os.environ["IMAGE"] = "2"
if "NOLOCALS" not in os.environ: os.environ["NOLOCALS"] = "1"
if "JIT_BATCH_SIZE" not in os.environ: os.environ["JIT_BATCH_SIZE"] = "0"

from tinygrad import fetch, Tensor, TinyJit, Context, GlobalCounters, Device
from tinygrad.helpers import DEBUG, getenv
from tinygrad.tensor import _from_np_dtype
from tinygrad.engine.realize import CompiledRunner

import onnx
from onnx.helper import tensor_dtype_to_np_dtype
from extra.onnx import get_run_onnx   # TODO: port to main tinygrad

OPENPILOT_MODEL = sys.argv[1] if len(sys.argv) > 1 else "https://github.com/commaai/openpilot/raw/v0.9.7/selfdrive/modeld/models/supercombo.onnx"
OUTPUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/openpilot.pkl"

def compile(onnx_file):
  onnx_model = onnx.load(onnx_file)
  Tensor.no_grad = True
  Tensor.training = False

  run_onnx = get_run_onnx(onnx_model)
  print("loaded model")

  input_shapes = {inp.name:tuple(x.dim_value for x in inp.type.tensor_type.shape.dim) for inp in onnx_model.graph.input}
  input_types = {inp.name: tensor_dtype_to_np_dtype(inp.type.tensor_type.elem_type) for inp in onnx_model.graph.input}
  # Float inputs and outputs to tinyjits for openpilot are always float32
  input_types = {k:(np.float32 if v==np.float16 else v) for k,v in input_types.items()}
  input_types = {k:np.uint8 if 'img' in k else v for k,v in input_types.items()}
  Tensor.manual_seed(100)
  new_inputs = {k:Tensor.randn(*shp, dtype=_from_np_dtype(input_types[k])).mul(8).realize() for k,shp in sorted(input_shapes.items())}
  new_inputs_numpy = {k:v.numpy() for k,v in new_inputs.items()}
  print("created tensors")

  run_onnx_jit = TinyJit(lambda **kwargs:
                         next(iter(run_onnx({k:v.to(Device.DEFAULT) for k,v in kwargs.items()}).values())).cast('float32'), prune=True)
  for i in range(3):
    GlobalCounters.reset()
    print(f"run {i}")
    inputs = {**{k:v.clone() for k,v in new_inputs.items() if 'img' in k},
              **{k:Tensor(v, device="NPY").realize() for k,v in new_inputs_numpy.items() if 'img' not in k}}
    with Context(DEBUG=max(DEBUG.value, 2 if i == 2 else 1)):
      ret = run_onnx_jit(**inputs).numpy()
    # copy i == 1 so use of JITBEAM is okay
    if i == 1: test_val = np.copy(ret)
  print(f"captured {len(run_onnx_jit.captured.jit_cache)} kernels")
  np.testing.assert_equal(test_val, ret, "JIT run failed")
  print("jit run validated")

  # checks from compile2
  kernel_count = 0
  read_image_count = 0
  gated_read_image_count = 0
  for ei in run_onnx_jit.captured.jit_cache:
    if isinstance(ei.prg, CompiledRunner):
      kernel_count += 1
      read_image_count += ei.prg.p.src.count("read_image")
      gated_read_image_count += ei.prg.p.src.count("?read_image")
  print(f"{kernel_count=},  {read_image_count=}, {gated_read_image_count=}")
  if (allowed_kernel_count:=getenv("ALLOWED_KERNEL_COUNT", -1)) != -1:
    assert kernel_count <= allowed_kernel_count, f"too many kernels! {kernel_count=}, {allowed_kernel_count=}"
  if (allowed_read_image:=getenv("ALLOWED_READ_IMAGE", -1)) != -1:
    assert read_image_count == allowed_read_image, f"different read_image! {read_image_count=}, {allowed_read_image=}"
  if (allowed_gated_read_image:=getenv("ALLOWED_GATED_READ_IMAGE", -1)) != -1:
    assert gated_read_image_count <= allowed_gated_read_image, f"too many gated read_image! {gated_read_image_count=}, {allowed_gated_read_image=}"

  with open(OUTPUT, "wb") as f:
    pickle.dump(run_onnx_jit, f)
  mdl_sz = os.path.getsize(onnx_file)
  pkl_sz = os.path.getsize(OUTPUT)
  print(f"mdl size is {mdl_sz/1e6:.2f}M")
  print(f"pkl size is {pkl_sz/1e6:.2f}M")
  print("**** compile done ****")
  return test_val

def test_vs_compile(run, new_inputs, test_val=None):
  new_inputs_numpy = {k:v.numpy() for k,v in new_inputs.items()}

  # create fake "from_blob" tensors for the inputs, and wrapped NPY tensors for the numpy inputs (these have the same underlying memory)
  inputs = {**{k:v for k,v in new_inputs.items() if 'img' in k},
            **{k:Tensor(v, device="NPY").realize() for k,v in new_inputs_numpy.items() if 'img' not in k}}

  # run 20 times
  for _ in range(20):
    st = time.perf_counter()
    out = run(**inputs)
    mt = time.perf_counter()
    val = out.numpy()
    et = time.perf_counter()
    print(f"enqueue {(mt-st)*1e3:6.2f} ms -- total run {(et-st)*1e3:6.2f} ms")
  print(out, val.shape, val.dtype)
  if test_val is not None: np.testing.assert_equal(test_val, val)
  print("**** test done ****")

  # test that changing the numpy changes the model outputs
  for v in new_inputs_numpy.values(): v *= 2
  out = run(**inputs)
  changed_val = out.numpy()
  np.testing.assert_raises(AssertionError, np.testing.assert_array_equal, val, changed_val)
  return val

def test_vs_onnx(new_inputs, test_val, onnx_file):
  new_inputs_numpy = {k:v.numpy() for k,v in new_inputs.items()}
  onnx_model = onnx.load(onnx_file)

  if getenv("ORT"):
    # test with onnxruntime
    import onnxruntime as ort
    onnx_session = ort.InferenceSession(onnx_file)
    onnx_output = onnx_session.run([onnx_model.graph.output[0].name], {k:v.astype(np.float16) for k,v in new_inputs_numpy.items()})
    new_torch_out = onnx_output[0]
    print("got ort outputs")
  else:
    # test with torch
    from test.models.test_onnx import run_onnx_torch
    # NOTE: we have to correct the order here
    new_torch_out = run_onnx_torch(onnx_model, {k.name:new_inputs_numpy[k.name] for k in onnx_model.graph.input}).numpy()
    print("got torch outputs")

  np.testing.assert_allclose(new_torch_out.reshape(test_val.shape), test_val, atol=1e-4, rtol=1e-2)
  print("test vs onnx passed")

if __name__ == "__main__":
  onnx_file = fetch(OPENPILOT_MODEL)
  test_val = compile(onnx_file) if not getenv("RUN") else None

  with open(OUTPUT, "rb") as f: pickle_loaded = pickle.load(f)

  # same randomness as compile
  Tensor.manual_seed(100)
  new_inputs = {nm:Tensor.randn(*st.shape, dtype=dtype).mul(8).realize() for nm, (st, _, dtype, _) in
                sorted(zip(pickle_loaded.captured.expected_names, pickle_loaded.captured.expected_st_vars_dtype_device))}

  test_val = test_vs_compile(pickle_loaded, new_inputs, test_val)
  if not getenv("FLOAT16"): test_vs_onnx(new_inputs, test_val, onnx_file)
