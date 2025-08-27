This folder provides the experiment configuration used for the publication:
"Can Human Drivers and Connected Autonomous Vehicles Co-exist in Lane-Free Traffic? A Microscopic Simulation Perspective"

How to generate and use the experiment:
- In the parent folder called "python_src". There is a script named "create_scenarios_TRC2025_mixed.py". Use it to generate simulation configurations under the folder "SUMO_Runs\ring_road_trc_2025\Experiments".
- The above script uses the file "SUMO_Runs\ring_road_trc_2025\trc_2025_default_settings.ini" as default parameter values.
- After running the script, there will also be a csv file "Experiments\trc_2025_experiments.csv" be generated. Use it together with the script "python_src\parallel_run.py" to start the experiments. Refer to the main README file of the repository to find out how to do that.