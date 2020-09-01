(define (domain blocksworld)

	(:predicates    (position       ?p)
	                (cube           ?c)
		            (at             ?c ?p)
		            (empty          ?p))

	(:action move :parameters (?cube ?from ?to)
		
		:precondition (and  (position       ?from)
		                    (position       ?to)
		                    (cube           ?cube)
		                    (at             ?cube ?from)
		                    (empty          ?to))

		:effect 	  (and  (at                 ?cube ?to)
		                    (empty              ?from)
		                    (not (at            ?cube ?from))
			                (not (empty         ?to))))
)	