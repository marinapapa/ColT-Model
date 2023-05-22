#ifndef ANALYSIS_OBS_HPP_INCLUDED
#define ANALYSIS_OBS_HPP_INCLUDED

#include <analysis/analysis.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>
#include <algorithm> 

namespace analysis
{
	template <typename Tag>
	class TimeSeriesObserver : public model::AnalysisObserver
	{
	public:
		TimeSeriesObserver(const std::filesystem::path& out_path, const json& J)
			: AnalysisObserver(out_path, J)
		{
			analysis::open_csv(outfile_stream_, full_out_path_, header_);
		}
		~TimeSeriesObserver() override {}


	protected:
		void notify_collect(const model::Simulation& sim) override
		{
			const auto tt = static_cast<float>(sim.tick()) * model::Simulation::dt();

			sim.visit_all<Tag>([&](auto& p, size_t idx) {
				// csv writing backwards, so vectors backwards from header, new element to be added in front
				//const auto& fi = sim.flocks<Tag>();										// all flocks
				const auto fl_id = sim.flock_of<Tag>(idx);
				const auto& thisflock = sim.flocks<Tag>()[fl_id];
				const auto dist2cent = glm::distance(p.pos, thisflock.gc()); // distance to center of flock
				const auto dir2fcent = glm::normalize(space::ofs(p.pos, thisflock.gc()));
				//const auto head_dev = glm::degrees(math::rad_between(p.dir, thisflock.vel));		 // deviation of self heading to flocks heading
				//const auto centr = centrality(p, idx, sim);
				//const auto rad2fcent = math::rad_between(p.dir, dir2fcent);

				//const auto nn = sim.sorted_view<Tag>(idx).cbegin(); // nearest neighbor
				data_out_.push_back({ dir2fcent.y, dir2fcent.x, dist2cent, static_cast<float>(p.get_current_state()), p.ang_vel, p.accel.y, p.accel.x, p.speed, p.dir.y,  p.dir.x,  p.pos.y, p.pos.x, static_cast<float>(idx), tt });
			});
		}

		void notify_save(const model::Simulation& sim) override
		{
			if (data_out_.empty()) { return; }
			std::cout << "Saving timeseries data.." << std::endl;
			analysis::export_data(data_out_, outfile_stream_);
		}

	private:
		const std::string header_ = "time,id,posx,posy,dirx,diry,speed,accelx,accely,ang_vel,state,dist2fcent,dirX2fcent,dirY2fcent";
	};

}

#endif
