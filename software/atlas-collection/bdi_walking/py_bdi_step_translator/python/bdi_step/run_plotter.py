import argparse
from bdi_step.translator import BDIStepTranslator, Mode

def run_plotter():
    t = BDIStepTranslator(mode=Mode.plotting)
    t.run()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Draw a rough estimate of the expected swing trajectory for BDI walk/step behaviors in the viewer")
    run_plotter()
