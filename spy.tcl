read_file -type verilog qr_decode.v a2i.v i2a.v

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
