MMRM (Multimedia Resource Manager) Testing Documentation
This test validates
1. Mmrm driver apis (register/register a client & set value)
2. Set value for different corners to overload mmrm driver & verify mmrm behavior for errors
3. Priority-based throttling in mmrm driver (TBD)

Usage: mmrm_test.sh [OPTIONS]
	Runs test to validate mmrm functionality.

OPTIONS:
	Supported options: TBD

TEST BEHAVIOR:
	* Verify register/deregister client multiple time without failure
	* set clk value & verify if it is configured correctly

TARGETS:
	* lahaina
