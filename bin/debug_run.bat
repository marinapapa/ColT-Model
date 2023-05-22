:: Runs debug version of model and calls R to plot the results
@echo OFF
echo Running model without the debugger..

:: Running model
%1 config=composed_config.json exp_files=true

:: R call if model exited normally
if errorlevel 0 ( 
	goto proc1
) 
goto proc2

:proc1
:: Plotting R
set input=run_R_plots_tmp.txt
if not exist %input% ( 
	echo R file missing - no plots produced from here. 
	goto proc3
)

for /F "usebackq tokens=*" %%A in (%input%) do %%A 
echo Deleting tmp..
del %input%

:: Analysis
:proc3
set input=run_R_analysis_tmp.txt
if not exist %input% ( 
	echo R file missing - no analysis run from here. 
	goto proc2 
)

for /F "usebackq tokens=*" %%A in (%input%) do %%A 
echo Deleting tmp..
del %input%

:proc2
echo All done!
exit 0      