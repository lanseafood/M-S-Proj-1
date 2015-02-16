/**
*	Created by Stefan Henneking on 01/12/15
*	GTid: 903085706
*
*	Georgia Institute of Technology, CSE 6730
*	Project 1: Road Traffic Simulation
*	trajectories.sql
**/

-- a. Create tables and import data
create table if not exists trajectories	( 	vehicle_id int, frame_id int, tot_frames int, epoch_ms int, local_x real, local_y real, 
											global_x real, global_y real, veh_len int, veh_wid int, veh_class int, veh_velocity real, 
											vehicle_acc real, lane_id int, org_zone int, dest_zone int, intersection int, section int,
											direction int, movement int, preceding_veh int, following_veh int, spacing real, headway real
										);

.separator ","
.import trajectories.csv trajectories

-- Build index
create index if not exists traj_vehicle_id_index on trajectories(vehicle_id);

-- Creating view (filter trajectory data)
create view if not exists vehicles as

	select 	 distinct t.vehicle_id,
			 t.org_zone,
			 t.dest_zone,
			 t.veh_len
	from	 trajectories t
	where 	 t.org_zone in (101, 102, 103, 106, 112, 113, 114, 115, 121, 122, 123)
			 and t.dest_zone in (201, 202, 203, 206, 212, 213, 214, 215, 221, 222, 223)
			 and t.dest_zone != ( t.org_zone + 100 )
;

select '';

-- Some user friendly output settings
.headers on
.mode column

-- Output rows, vehicles of original trajectory data
select 	count(t.vehicle_id) as "Number of Rows",
		count(distinct t.vehicle_id) as "Number of Vehicles"
from trajectories t
;

.headers off
select'';
.headers on

-- Output number of vehicles of filtered trajectory data
select 	count(t.vehicle_id) as "Number of Vehicles (filtered origin and destination zones)"
from vehicles t
;

.headers off
select'';
.headers on

-- Output average vehicle length
select avg(veh_len) as "Average Vehicle Length"
from vehicles
;

.headers off
select'';
.headers on

-- Output origin zone probability distribution
select	v.org_zone as "Origin Zone",
		count(v.vehicle_id) as "Vehicles",
		round(100.0*count(v.vehicle_id) / ( select count(vc.vehicle_id) from vehicles vc ), 2) || '%' as "Probability"
from vehicles v
group by v.org_zone
;

.headers off
select'';
.headers on

-- Output destination zone probability distribution
select	v.dest_zone as "Destination Zone",
		count(v.vehicle_id) as "Vehicles",
		round(100.0*count(v.vehicle_id) / ( select count(vc.vehicle_id) from vehicles vc ), 2) || '%' as "Probability"
from vehicles v
group by v.dest_zone
;

.headers off
select'';
.headers on

-- Output destination zone probability distribution dependent on origin
select 	a."Origin Zone" as "Origin Zone", 
		a."Destination Zone" as "Destination Zone",
		a."Vehicles" as "Vehicles",
		round(100.0 * a."Vehicles" / 
						( 	select count(vehicle_id) 
						 	from vehicles vc_b 
						 	where vc_b.org_zone = a."Origin Zone" 
						 	group by vc_b.org_zone 
						)
		, 2) || '%' as "Probability per Origin"
from	( 	select 	vc_a.org_zone as "Origin Zone",
					vc_a.dest_zone as "Destination Zone",
					count(vc_a.vehicle_id) as "Vehicles"
			from 	vehicles vc_a
			group by vc_a.org_zone, vc_a.dest_zone
		) a
order by a."Origin Zone", a."Destination Zone"
;

-- Clean Up
drop view  if exists vehicles;
drop table if exists trajectories;

/* eof */
