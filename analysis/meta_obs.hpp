#ifndef META_OBS_HPP_INCLUDED
#define META_OBS_HPP_INCLUDED

#include <analysis/analysis.hpp>
#include <analysis/analysis_obs.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>
#include <analysis/diffusion_obs.hpp>


namespace analysis
{
	template <typename Tag>
	class SnapShotObserver : public model::Observer
	{
	public:
		SnapShotObserver(const std::filesystem::path& out_path, const json& J)
		{
			const std::string out_name = J["output_name"];
			full_out_path_ = out_path / out_name;
			n_ = 0;
		}
		~SnapShotObserver() override {}

	protected:
		void notify_once(const model::Simulation& sim) override
		{
			notify_collect(sim);

			if (data_out_.empty()) { return; }

			const auto filepath = full_out_path_.string() + "_" + std::to_string(n_) + ".csv";
			const std::string header = "id,posx,posy,dirx,diry,speed,accelx,accely";
			analysis::open_csv(outfile_stream_, filepath, header);
			notify_save(sim, outfile_stream_);
			outfile_stream_.close();
		}

		void notify_collect(const model::Simulation& sim)
		{
			sim.visit_all<Tag>([&](auto& p, size_t idx) {
				// csv writing backwards, so vectors backwards from header, new element to be added in front
   			data_out_.push_back({ p.accel.y, p.accel.x, p.speed, p.dir.y,  p.dir.x,  p.pos.y, p.pos.x, static_cast<float>(idx) });
  		});
		}

		void notify_save(const model::Simulation& sim, std::ofstream& outFile)
		{
			if (n_param_ > data_out_[0].size()) {
				std::cout << "Warning: size of saving vector lower than defined, data wont be saved." << std::endl;
				data_out_.clear();
				return;
			}
			std::cout << "Taking data snapshot.." << std::endl;
			analysis::export_data<n_param_>(data_out_, outFile);
			outFile.close();
			++n_;
			data_out_.clear();
		}

	private:
		std::deque<std::array<float, 9>> data_out_; // idx, pos.x, pos.y, dir.x, dir.y, speed, accel.x, accel.y
		static const size_t n_param_ = 9; // number of parameters to save in timeseries
		std::filesystem::path full_out_path_;
		size_t n_; // number of snapshots taken
		std::ofstream outfile_stream_;
	};


	class DataExpObserver : public model::Observer
	{
	public:
		DataExpObserver(const json& J)
		{
			auto& ja = J["Simulation"]["Analysis"];

			const std::string outf = ja["output_path"];

			json_ext_ = ja["Externals"];
			json_ext_["output_path"] = outf;

			const std::string confname = json_ext_["configName"];
			std::ofstream config_file_id(outf + "/" + confname + ".txt"); // create config id as name of empty file in folder

			save_json(J, (path_t(outf) / "composed_config.json"));
		}
		~DataExpObserver() {}

	protected:
		void notify(long long lmsg, const model::Simulation& sim)
		{
			using Msg = model::Simulation::Msg;
			auto msg = Msg(lmsg);

			switch (msg) {
			case Msg::Tick: {
				//const auto& x = sim.pop<starling_tag>()[0];
				//std::cout << x.circumcenter().R << '\n';
				break;
			}
			case Msg::Finished:
				notify_save(sim);
				break;
			default:
				break;
			}
		}

		void notify_save(const model::Simulation& sim)
		{
			if (int(json_ext_["plot?"]) != 0)
			{
				analysis::plot_data_bash(json_ext_);
			}
		}

	private:
		json json_ext_;
	};


	template <typename Tag>
	std::vector<std::unique_ptr<Observer>> CreateObserverChain(json& J)
	{
		auto& ja = J["Simulation"]["Analysis"];
		auto& jstarling = J["Starling"]["states"];
		auto& N = J["Starling"]["N"];
		std::vector<std::unique_ptr<Observer>> res;

		if (ja.size() == 0 || ja["data_folder"] == "")
		{
			std::cout << "No analysis observers created, data extraction will not take place." << std::endl;
			return res; // no observers created
		}
		const auto unique_path = analysis::unique_output_folder(ja);

		// inject output path to json object
		ja["output_path"] = unique_path.string();

		const auto& jo = ja["Observers"];
		for (const auto& j : jo)
		{
			std::string type = j["type"];
			if (type == "TimeSeries") res.emplace_back(std::make_unique<TimeSeriesObserver<Tag>>(unique_path, j));
			else if (type == "SnapShot") res.emplace_back(std::make_unique<SnapShotObserver<Tag>>(unique_path, j));
			else if (type == "Diffusion") res.emplace_back(std::make_unique<DiffusionObserver<Tag>>(unique_path, j));
			else throw std::runtime_error("unknown observer");
		}
		res.emplace_back(std::make_unique<DataExpObserver>(J)); // has to be at the end of the chain
		return res;
	}
}

#endif
