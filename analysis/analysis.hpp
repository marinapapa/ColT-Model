#ifndef ANALYSIS_HPP_INCLUDED
#define ANALYSIS_HPP_INCLUDED

#include <iostream>
#include <filesystem>
#include <deque>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <model/flock.hpp>
#include <model/simulation.hpp>
#include <libs/math.hpp>
#include <libs/space.hpp>


#ifdef _WIN32
constexpr const char* RSCRIPT_BINARY = "RScript.exe";
#else
constexpr const char* RSCRIPT_BINARY = "rscript";
#endif


using namespace model;

namespace analysis
{
	using timeseries_t = std::deque<std::vector<float>>;
	using path_t = std::filesystem::path;

	template <typename Agent>
  inline float head_dif(const Agent& p, const model::flock_descr& f)
  {
		return math::rad_between_xy(p.dir, f.vel);
  }

	template <typename Agent>
	inline float centrality(const Agent& pf, const size_t& idxf, const model::Simulation& sim)
	{
		//const auto sv = sim.sorted_view<starling_tag>(idxf);
		//const auto& flock = sim.pop<starling_tag>();
		//vec3 adir(0.f);
		//auto n = 0.f; // number of neighbors
		//for (auto it = sv.cbegin(); it != sv.cend(); ++it) {
		//	if (sim.flock_of<starling_tag>(idxf) == sim.flock_of<starling_tag>(it->idx))
		//	{
		//		adir += space::ofs(pf.pos, flock[it->idx].pos);
		//		++n;
		//	}
		//}

		vec3 adir(0.f);
		auto n = 0.f; // number of neighbors
		sim.visit_all<starling_tag>([&](auto& p, size_t idx) {
			if (idx != idxf) {
				if (sim.flock_of<starling_tag>(idxf) == sim.flock_of<starling_tag>(idx)) {
					adir += space::ofs(pf.pos, p.pos);
					++n;
				}	
			}
		});

		if (n) {
			return glm::length(adir / n);
		}
		return 0.f;
	}

	inline path_t output_path(const json& J)
	{
		auto exePath = std::filesystem::current_path();
		path_t top_folder = exePath;

		// DECIDE ON SAVING REPO - NOW GOES TO BIN

		//char* buf = nullptr;
		//size_t sz = 0;
		//if (_dupenv_s(&buf, &sz, "dancesPATH") == 0 && buf != nullptr)
		//{
		//	top_folder = buf;
		//	free(buf);
		//}

		std::string main_dat_folder = "sim_data";
		std::string outf = J["data_folder"];

		path_t filefolder = (top_folder / main_dat_folder).string();
		if (!(std::filesystem::exists(filefolder)))
		{
			std::filesystem::create_directory(filefolder.string());
		}
		filefolder = (filefolder / outf).string();
		if (!(std::filesystem::exists(filefolder)))
		{
			std::filesystem::create_directory(filefolder.string());
		}
		return filefolder;
	}

	inline path_t unique_output_folder(const json& J)
	{
		auto distr = std::uniform_int_distribution<int>(0, 1000);
		const auto random_id = std::to_string(distr(model::reng));
		const time_t now = time(0);
		struct tm* local_time = localtime(&now);

		const std::string thyear = std::to_string(1900 + local_time->tm_year);
		const std::string thmonth = std::to_string(1 + local_time->tm_mon) + std::to_string(local_time->tm_mday);
		const std::string thtime = std::to_string(local_time->tm_hour) + std::to_string(local_time->tm_min) + std::to_string(local_time->tm_sec);
		const std::string full_name = thyear + thmonth + thtime + std::to_string(now) + random_id;

		const auto outf = output_path(J);
		const path_t filefolder = (outf / full_name).string();
		if (!(std::filesystem::exists(filefolder))) {
			std::filesystem::create_directory(filefolder);
		}
		return filefolder;
	}

	template < size_t P, typename VecType >
	struct do_write_csv 
	{
		static void write_in_csv(std::ofstream& outFile, VecType& i) // or std::deque<std::vector<float>>::const_iterator
		{
			outFile << i[P-1] ;
			outFile << ',';
			do_write_csv<P - 1, VecType>::write_in_csv(outFile, i);
		}
	};

	template< typename VecType >
	struct do_write_csv<1, VecType>
	{
		static void write_in_csv(std::ofstream& outFile, VecType& i)
		{
			outFile << i[1] << std::endl;
		}
	};


	inline void open_csv(std::ofstream& outFile, const std::string& full_path, const std::string& header)
  {
    outFile.open(full_path);
    outFile << header << std::endl;
  }

  template < size_t P, typename DecType>
	void export_data(const DecType& data_out, std::ofstream& outFile)
  {
    if (P != data_out[0].size()) { std::cout << "Warning: size of saving vector different that defined, data might be missing." << std::endl; }

    for (auto i : data_out)
    {
      do_write_csv<P, typename DecType::value_type>::write_in_csv(outFile, i);
    }
  }

	template <typename DecType>
	void export_data(const DecType& data_out, std::ofstream& outFile)
	{
		for (auto i : data_out)
		{
			size_t p = i.size() - 1;
			while (!(i.empty()) && p != 0)
			{
				outFile << i[p] << ',';
				i.pop_back();
				--p;
			}
			outFile << i[0] << std::endl;
		}
	}


	template < size_t P, typename DecType>
	inline void export_csv(const DecType& data_out, const std::string& full_path, const std::string& header)
	{
		if (P != data_out[0].size()) { std::cout << "Warning: size of saving vector different that defined, data might be missing." << std::endl; }
		
		std::ofstream outFile;

		outFile.open(full_path);
		outFile << header << std::endl;
		for (auto i : data_out) 
		{
			do_write_csv<P, typename DecType::value_type>::write_in_csv(outFile, i);
		}
		outFile.close();
	}

	template < size_t P, typename DecType>
	inline void export_csv(const DecType& data_out, const std::string& full_path)
	{
		if (P != data_out[0].size()) { std::cout << "Warning: size of saving vector different that defined, data might be missing." << std::endl; }

		std::ofstream outFile;

		outFile.open(full_path, std::ios_base::app);
		for (auto i : data_out)
		{
			do_write_csv<P, typename DecType::value_type>::write_in_csv(outFile, i);
		}
		outFile.close();
	}

	inline void plot_data_bash(const json& J)
	{
#ifndef PIGEON_DEBUG
		// Fix file names
		const auto exePath = std::filesystem::current_path(); 
		const std::string rs = J["Rscript"];
		const std::string outf = J["output_path"];

		const std::filesystem::path rscript = (exePath / rs).string();
		const std::string saving_path = (exePath / outf ).string();

		//std::string plotname;
		//plotname.assign(csvname);
		//plotname.replace(plotname.begin() + plotname.find(".csv"), plotname.end(), "_plots.pdf");
		//plotname = (exePath / outf / plotname).string();

		// run R script 
		std::ofstream outFile;
		outFile.open("run_R_plots_tmp.txt");
		outFile << RSCRIPT_BINARY << ' ' << (exePath / rs).string() << ' '
			<< saving_path ;
		outFile.close();
#endif
	}

}
#endif
