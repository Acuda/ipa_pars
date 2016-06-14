(define (domain cob-test-domain-01)
	(:requirements :strips :typing :adl )
	(:types room loc trash-bin - location
		box agent room-clean-tasks dirt tool - phys-obj
		robot user - agent
		vac-cleaner - tool
	)
	(:predicates
		(trans ?location-1 ?location-2 - room)
		(at ?what - phys-obj ?location - room)
		(inspected ?where - room)
		(cleared ?trash - trash-bin)
		(cleaned ?dirt - dirt)
		(reached-trash ?trash - trash-bin)
		(reached-dirt-loc ?dirt - dirt)
		(verify-cleaning-success ?dirt - dirt)
		(all-rooms-inspected ?who - robot)
		(ready-for-dirtmap ?who - robot ?what - tool)
		(ready-for-vacuum ?who - robot)
		(detect-trash-bins-on ?where - room)
		(detect-dirt-on ?where - room)
		(process-results ?who - robot)
	)

	(:action change-tool
		:parameters (?who - robot ?vac-cleaner - tool)
		:precondition (and 
				 (forall (?ro - room) (inspected ?ro))
				)
		:effect (and
			    (ready-for-dirtmap ?who ?vac-cleaner)
			)
	)

	(:action process-cleaning-results
		:parameters (?who - robot)
		:precondition (and
				  (forall (?d - dirt) (cleaned ?d)))
		:effect (and (process-results ?who))
	)
	
	(:action get-dirt-map
		:parameters (?who - robot ?vac-cleaner - tool)
		:precondition (and
				 (ready-for-dirtmap ?who ?vac-cleaner)
			      )
		:effect (and
			     (ready-for-vacuum ?who)
			)
	)

	(:action clean-location
		:parameters (?who - robot ?where - location ?dirt - dirt)
		:precondition (and
				 (ready-for-vacuum ?who)
				 (at ?who ?where)
				 (at ?dirt ?where)
				)
		:effect (and (cleaned ?dirt))
	)
	
	(:action verify-cleaning
		:parameters (?who - robot ?where - location ?dirt - dirt)
		:precondition (and
				(at ?who ?where)
				(at ?dirt ?where)
				(reached-dirt-loc ?dirt)
				)
		:effect (and (verify-cleaning-success ?dirt)
			     (not (reached-dirt-loc ?dirt))
			)
	)

	(:action move-to-inspect-location
		:parameters (?who - robot ?where - location ?dirt - dirt)
		:precondition (and
				 (at ?who ?where)
				 (at ?dirt ?where)
				 (cleaned ?dirt)
				)
		:effect (and (reached-dirt-loc ?dirt))
	)

	(:action inspect-room
		:parameters (?who - robot ?where - room)
		:precondition (and
				   (at ?who ?where)
				   (not (inspected ?where))
				   (detect-trash-bins-on ?where)
				   (detect-dirt-on ?where)
				)	
		:effect (and (inspected ?where))
	)
	
	(:action move-to-trash-bin
		:parameters (?who - robot ?where - room ?trash - trash-bin)
		:precondition (and
				 (at ?who ?where)
				 (at ?trash ?where)
				 (inspected ?where)
				 (not (cleared ?trash))
				)
		:effect (and (reached-trash ?trash)
			)
	)

	(:action clear-trash-bin
		:parameters (?who - robot ?where - room ?trash - trash-bin)
		:precondition (and
				 (at ?who ?where)
			         (at ?trash ?where)
				 (inspected ?where)
				 (reached-trash ?trash)
				 (not (cleared ?trash))
				)
		:effect (and (cleared ?trash)
			)
	)
	
	(:action detect-trash-bins-off
		:parameters (?who - robot ?where - room)
		:precondition (and
				   (at ?who ?where)
				   (detect-trash-bins-on ?where)
				)
		:effect (and (not (detect-trash-bins-on ?where))
			)
	)

	(:action move-robo-to
		:parameters (?who - robot ?from - room ?to - room)
		:precondition (and 
				   (at ?who ?from)
				   (trans ?from ?to)
				   (not (detect-dirt-on ?from))
				   (not (detect-trash-bins-on ?from))
				   )
		:effect (and (not (at ?who ?from))
			     (at ?who ?to))
	)

			     (at ?who ?to)
			     )
	)
)
