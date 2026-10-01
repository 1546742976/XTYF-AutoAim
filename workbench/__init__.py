"""Read-only source integration for the offline experiment workbench."""

import sys

# Running the workbench must not create bytecode in the source checkout.
sys.dont_write_bytecode = True
