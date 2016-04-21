(define (domain cob-test-domain-01)
	(:requirements :strips)
	(:predicates
		(trans ?location-1 ?location-2)
		(at ?what ?location)
		(object-is-at ?what ?where) 
		(have ?who ?what) 
		(is-robo ?who)
		(see ?who ?what)
		(positioned-near ?who ?where)
	)

	(:action move-robo-to
		:parameters (?who ?from ?to)
		:precondition (and 
				   (is-robo ?who)
				   (at ?who ?from)
				   (trans ?from ?to))
		:effect (and (not (at ?who ?from))
			     (at ?who ?to))
	)

	(:action take
		:parameters (?who ?what ?where)
		:precondition (and (is-robo ?who)
				   (at ?who ?where)
				   (at ?what ?where)
				   (see ?who ?what))

		:effect (and (have ?who ?what)
			     (not (at ?what ?where)))
	)

	(:action look-at
		:parameters (?who ?what ?where)
		:precondition (and (is-robo ?who)
				   (at ?who ?where)
				   (at ?what ?where)
				   )
		:effect (and (see ?who ?what))
	)

	(:action position-in-front
		:parameters (?who ?what ?where)
		:precondition (and (is-robo ?who)
				   (at ?who ?where)
				   (at ?what ?where)
				   )
		:effect (and (positioned-near ?who ?where))
	)

	(:action grip-object
		:parameters (?who ?what ?where ?is-in-position)
		:precondition (and (is-robo ?who)
				   (at ?who ?where)
				   (at ?what ?where)
				   (positioned-near ?is-in-position)
				   )
		:effect (and (have ?who ?what)
				(not (at ?what ?where)))
	)
	
	(:action tell-user-current-state
		:parameters (?who ?state-transition-happend)
		:precondition (and (is-robo ?who)
				   (state-is-new ?state-transition-happend)
				)
		:effect (state-transition-happend)
)	
