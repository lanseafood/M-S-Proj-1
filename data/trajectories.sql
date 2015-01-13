/*
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

-- Creating views
create view if not exists vehicles as

	select 	 distinct t.vehicle_id,
			 t.org_zone,
			 t.dest_zone
	from	 trajectories t
;

select '';

-- Some user friendly output settings
.headers on
.mode column

-- Output
select 	count(t.vehicle_id) as "Number of Rows",
		count(distinct t.vehicle_id) as "Number of Vehicles"
from trajectories t
;

.headers off
select'';
.headers on

select	v.org_zone as "Origin Zone",
		count(v.vehicle_id) as "Vehicles",
		round(100.0*count(v.vehicle_id) / ( select count(vc.vehicle_id) from vehicles vc ), 2) || '%' as "Probability"
from vehicles v
group by v.org_zone
;

.headers off
select'';
.headers on

select	v.dest_zone as "Destination Zone",
		count(v.vehicle_id) as "Vehicles",
		round(100.0*count(v.vehicle_id) / ( select count(vc.vehicle_id) from vehicles vc ), 2) || '%' as "Probability"
from vehicles v
group by v.dest_zone
;

-- Clean Up
drop view  if exists vehicles;
drop table if exists trajectories;

/* eof */