read_file -type verilog *.v

current_goal lint/lint_rtl -alltop

run_goal
capture moresimple_all.rpt {write_report moresimple}

current_goal lint/lint_turbo -alltop
run_goal
capture -append moresimple_all.rpt {write_report moresimple}

current_goal lint/lint_functional_rtl -alltop
run_goal
capture -append moresimple_all.rpt {write_report moresimple}

current_goal lint/lint_abstract -alltop
run_goal
capture -append moresimple_all.rpt {write_report moresimple}
