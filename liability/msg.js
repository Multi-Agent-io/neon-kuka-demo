import Web3 from 'web3';
import { readFileSync } from "fs";
//import { off } from 'process';

const web3 = new Web3("https://proxy.devnet.neonlabs.org/solana")
let args = process.argv.slice(2); //path to controller directory
let config = readFileSync(`${args[0]}/config/config`)
let json_config = JSON.parse(config)



const liability_address = json_config["liability_contract"]


async function randomDemand(web3) {
    let account = json_config["account"]
    let private_key = json_config["private_key"]
    let demand =
        { model:        web3.utils.randomHex(34)
        , objective:    web3.utils.randomHex(34)
        , token:        xrt_address //xrt address
        , cost:         1
        , validator:    '0x0000000000000000000000000000000000000000'
        , validatorFee: 0
        , deadline:     await web3.eth.getBlockNumber() + 100000
        , sender:       account
        };

    const hash = web3.utils.soliditySha3(
        {t: 'bytes',   v: demand.model},
        {t: 'bytes',   v: demand.objective},
        {t: 'address', v: demand.token},
        {t: 'uint256', v: demand.cost},
        {t: 'address', v: demand.validator},
        {t: 'uint256', v: demand.validatorFee},
        {t: 'uint256', v: demand.deadline},
        {t: 'address', v: demand.sender}
    );
    demand.signature = await web3.eth.accounts.sign(hash, private_key);

    return demand;
}

let demand = await randomDemand(web3)

async function pairOffer(demand, web3) {
    const robot_private_key = json_config["robot_private_key"]
    const robot_address = json_config["robot_address"]

    let offer =
        { model:        demand.model
        , objective:    demand.objective
        , token:        demand.token
        , cost:         demand.cost
        , validator:    demand.validator
        , deadline:     await web3.eth.getBlockNumber() + 100000
        , sender:       robot_address
        };

    const hash = web3.utils.soliditySha3(
        {t: 'bytes',   v: offer.model},
        {t: 'bytes',   v: offer.objective},
        {t: 'address', v: offer.token},
        {t: 'uint256', v: offer.cost},
        {t: 'address', v: offer.validator},
        {t: 'uint256', v: offer.deadline},
        {t: 'address', v: offer.sender}
    );
    offer.signature = await web3.eth.accounts.sign(hash, robot_private_key);

    return offer;
}

let offer = await pairOffer(demand, web3)


async function liabilityCreation(web3) {
    let account = json_config["account"]
    let robot_account = json_config["robot_address"]

    let abi = readFileSync(`${args[0]}/liability/abi/Liability.json`)
    let json_abi = JSON.parse(abi)
    //console.log(json_abi)

    let liability = await new web3.eth.Contract(json_abi, liability_address)
   // let tx = await liability.setup(xrt_address, {from: account})
    //console.log(tx)

    console.log("deamand")
    // console.log(demand)
    let tx = await liability.methods.demand(
        demand.model,
        demand.objective,
        demand.token,
        demand.cost,
        demand.validator,
        demand.validatorFee,
        demand.deadline,
        demand.sender,
        demand.signature.signature
        ).send({from: account, gas: 1000000000})

    console.log(tx.transactionHash)

    await new Promise(resolve => setTimeout(resolve, 2000));
    
    console.log("offer")
    // console.log(offer)
    let tx_offer = await liability.methods.offer(
        offer.model,
        offer.objective,
        offer.token,
        offer.cost,
        offer.validator,
        offer.deadline,
        offer.sender,
        offer.signature.signature
    ).send({from: robot_account, gas: 1000000000})
  
    console.log(tx_offer.transactionHash)
}

await web3.eth.accounts.wallet.add(json_config["private_key"])
await web3.eth.accounts.wallet.add(json_config["robot_private_key"])
web3.eth.defaultAccount = json_config["account"]

await liabilityCreation(web3)
