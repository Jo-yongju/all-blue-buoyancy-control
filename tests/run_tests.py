from pathlib import Path
import os, subprocess, shutil
R=Path(__file__).resolve().parents[1]
os.chdir(R)
R=Path('.')
S=R/'firmware/updown_dir_control/Core/Src'; I=S.parent/'Inc'; T=R/'tests'
O=R/'build/host'; O.mkdir(parents=True,exist_ok=True)
gcc=os.environ.get('CC') or shutil.which('gcc')
if not gcc: raise SystemExit('GCC not found. Set CC or add gcc to PATH.')
base=[gcc,'-std=c11','-Wall','-Wextra','-Werror','-I'+str(T),'-I'+str(I)]
def run(args,**kw): subprocess.run(args,check=True,**kw)
run(base+['-DLiveIO_Off=Hardware_Off','-DLiveIO_InitOutputs=Hardware_InitOutputs','-DLiveIO_Write=Hardware_Write','-c',str(S/'mdd10a_output.c'),'-o',str(O/'output.o')])
sources=['motor_test.c', 'motor_command.c', 'control_app.c', 'console_service.c', 'app_transport.c', 'app_inputs.c', 'encoder_service.c', 'team_link.c', 'dual_position_bench.c', 'console_queue.c', 'encoder_observer.c', 'ibus_frame.c', 'input_trial.c', 'periodic_job.c', 'position_control.c', 'receiver_command.c', 'receive_queue.c', 'slider_calibration.c', 'syringe_control.c', 'team_input.c']
exe=O/('integration.exe' if os.name=='nt' else 'integration')
run(base+[str(T/'test_motor_integration.c'),*[str(S/f) for f in sources],str(T/'queue_fake.c'),str(T/'encoder_fake.c'),str(O/'output.o'),'-lm','-o',str(exe)])
for mode in ['TEST_MDD10A','TEST_MDD_POSITION','TEST_MDD2_POSITION','TEST_DUAL']:
    env=os.environ.copy()
    for key in ['TEST_MDD10A','TEST_MDD_POSITION','TEST_MDD2_POSITION','TEST_DUAL']: env.pop(key,None)
    env[mode]='1'; print(mode,flush=True); run([str(exe)],env=env)
seq=O/('sequence.exe' if os.name=='nt' else 'sequence')
run(base+[str(T/'test_sequence.c'),*[str(S/f) for f in ['team_input.c','receiver_command.c','syringe_control.c','slider_calibration.c']],'-lm','-o',str(seq)])
run([str(seq)])
