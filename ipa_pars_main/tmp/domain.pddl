(define (domain cob-test-domain-01)
<<<<<<< HEAD
 	(:requirements :strips :typing :adl )
 	(:types room loc trash-bin - location
 		box agent room-clean-tasks dirt tool - phys-obj
 		robot user - agent
 		vac-cleaner - tool
=======
 	(:requirements :strips :typing)
 	(:types room phys-obj gripper
 		box agent - phys-obj
 		robot user - agent
>>>>>>> github-ce/indigo_dev
 	)
 	(:predicates
 		(trans ?location-1 ?location-2 - room)
 		(at ?what - phys-obj ?location - room)
<<<<<<< HEAD
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
=======
 		(have ?who - agent ?what - phys-obj)
 		(occupied ?where - room)
 	(see ?who - robot ?what - phys-obj)
 	(neglected ?who - robot)
>>>>>>> github-ce/indigo_dev
 
 	;;; gripper
 	(which-gripper ?x - gripper)
 	(gripper-free ?x - gripper)
 	(carry ?what - phys-obj ?withwhat - gripper)
 	)
 
 	(:action move-robo-to
 		:parameters (?who - robot ?from - room ?to - room)
 		:precondition (and 
 				   (at ?who ?from)
 				   (trans ?from ?to)
 				   (not (occupied ?to))
 				   (neglected ?who)
 				   )
 		:effect (and (not (at ?who ?from))
<<<<<<< HEAD
 			     (at ?who ?to))
 	)
=======
 			     (at ?who ?to)
 			     (neglected ?who)
 			     )
 	)
 	
 	;;;(:action move-user
 	;;;	:parameters (?who ?from ?to)
 	;;;	:precondition (and
 	;;;			 (is-user ?who)
 	;;;			 (at ?who ?from)
 	;;;			 (trans ?from ?to)
 	;;;			 (not (occupied ?to))
 	;;;		      )
 	;;;	:effect (and (not (at ?who ?from))
 	;;;		     (at ?who ?to)
 	;;;		)
 	;;;)
 
 	;;;(:action be-close-to
 	;;;	:parameters (?who ?from ?close-to)
 	;;;	:precondition (and
 	;;;			  (is-robo ?who)
 	;;;			  (at ?who ?from)
 	;;;			  (trans ?from ?close-to)
 	;;;			  )
 	;;;	:effect: (and (nearby ?who ?close-to)
 	;;;)
 
 	;;;(:action move-robo-to
 	;;;	:parameters (?who ?from ?to)
 	;;;	:precondition (and
 	;;;			   (is-robo ?who)
 	;;;			   (at ?who ?from)
 	;;;			   (trans ?from ?to))
 	;;;	:effect (and (not (at ?who ?from))
 	;;;			(at ?who ?to))
 	;;;)
 
 	;;;(:action take
 	;;;	:parameters (?who - robot ?what - phys-obj ?where - room ?close-to - room)
 	;;;	:precondition (and ;;;(is-robo ?who)
 	;;;			   (at ?who ?where)
 	;;;			   (at ?what ?close-to)
 	;;;			   (trans ?where ?close-to)
 	;;;			   ;;;(nearby ?who ?close-to)
 	;;;			   )
 	;;;	:effect (and (have ?who ?what)
 	;;;		     (not (at ?what ?close-to))
 	;;;		     ;;;(gripper arm-left)
 	;;;		     ;;;(carry ?what arm-left)
 	;;;		)
 	;;;)
 
 
 	
 	(:action look-at
 		:parameters (?who - robot ?what - phys-obj ?where - room ?close-to - room)
 		:precondition (and
 				   (at ?who ?where)
 				   (trans ?where ?close-to)
 				   (at ?what ?close-to)
 				   (neglected ?who)
 				   )
 		:effect (and (see ?who ?what)
 			     (not (neglected ?who))
 			)
 	)
 
 	(:action grip-it
 		:parameters (?who - robot ?what - phys-obj ?where - room ?from - room ?withwhat - gripper)
 		:precondition (and 
 				   (at ?who ?where)
 				   (see ?who ?what)
 				   (trans ?where ?from)
 				   (at ?what ?from)
 				   (not (neglected ?who))
 	
 				   ;;; gripper
 				   (which-gripper ?withwhat)
 				   (gripper-free ?withwhat)
 				   )
 		:effect (and 	(neglected ?who)
 				
 				(carry ?what ?withwhat)
 				(not (occupied ?from))
 				(not (at ?what ?from))
 				(not (see ?who ?what))
 				(not (gripper-free ?withwhat))
 			)
 	)
 	
 	(:action put-it
 		:parameters (?who - robot ?what - phys-obj ?where - room ?to - room ?withwhat - gripper)
 		:precondition (and ;;;(is-robo ?who)
 				   ;;;(is-user ?pers)
 				   (at ?who ?where)
 				   ;;;(have ?who ?what)
 				   (trans ?where ?to)
 				   ;;;(at ?pers ?where)
 				   (which-gripper ?withwhat)
 				   (carry ?what ?withwhat)
 				)
 		:effect (and (at ?what ?to)
 			      (occupied ?to)
 			      ;;;(not (have ?who ?what))
 			      (gripper-free ?withwhat)
 			      (not (carry ?what ?withwhat))
 			)
 	)
 	
 	(:action deliver-to
 		:parameters (?pers - user ?what - phys-obj ?who - robot ?where - room ?to - room ?withwhat - gripper)
 		:precondition (and
 				   (at ?pers ?to)
 				   (at ?who ?where)
 				   (trans ?where ?to)
 				   (see ?who ?pers)
 				   (not (neglected ?who))
 				   (which-gripper ?withwhat)
 				   (carry ?what ?withwhat)
 				)
 		:effect (and (have ?pers ?what)
 			     (neglected ?who)
 			     (not (see ?who ?what))
 			     (gripper-free ?withwhat)
 			     (carry ?what ?withwhat)
 			)
 	)
 
 	(:action neglect
 		:parameters (?pers - user ?who - robot )
 		:precondition (and
 				   (not (neglected ?who))
 					)
 		:effect (and (neglected ?who)
 			     (not (see ?who ?pers))
 			)
 	)
 
>>>>>>> github-ce/indigo_dev
 )
