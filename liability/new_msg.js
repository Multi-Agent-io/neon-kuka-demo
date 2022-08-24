import Web3 from 'web3';
import { readFileSync } from "fs";
import BigNumber from "bignumber.js";
//import { off } from 'process';

const web3 = new Web3("https://proxy.devnet.neonlabs.org/solana")
let args = process.argv.slice(2); //path to controller directory
let config = readFileSync(`${args[0]}/config/config`)
let json_config = JSON.parse(config)

const xrt_contract = json_config["xrt_contract"]
const lighthouse_address = json_config["lighthouse_address"]
const factory_address = json_config["factory_address"]
const validator_address = json_config["validator_address"]

const main_account = json_config["main_account"]
const main_account_pk = json_config["main_account_pk"]

const robot_account = json_config["robot_account"]
const robot_account_pk = json_config["robot_account_pk"]

const validator_account_pk = json_config["validator_account_pk"]


async function randomDemand(web3, factory) {
    let demand =
    {
        model: web3.utils.randomHex(34)
        , objective: web3.utils.randomHex(34)
        , token: xrt_contract //xrt address
        , cost: 1
        , lighthouse: lighthouse_address
        , validator: validator_address
        , validatorFee: 1
        , deadline: await web3.eth.getBlockNumber() + 100000
        , nonce: BigNumber(await factory.methods.nonceOf(main_account).call()).toNumber()
        , sender: main_account
    };

    const hash = web3.utils.soliditySha3(
        { t: 'bytes', v: demand.model },
        { t: 'bytes', v: demand.objective },
        { t: 'address', v: demand.token },
        { t: 'uint256', v: demand.cost },
        { t: 'address', v: demand.lighthouse },
        { t: 'address', v: demand.validator },
        { t: 'uint256', v: demand.validatorFee },
        { t: 'uint256', v: demand.deadline },
        { t: 'uint256', v: demand.nonce },
        { t: 'address', v: demand.sender }
    );
    demand.signature = await web3.eth.accounts.sign(hash, main_account_pk);

    return demand;
}

async function pairOffer(demand, web3, factory) {
    let offer =
    {
        model: demand.model
        , objective: demand.objective
        , token: demand.token
        , cost: demand.cost
        , validator: demand.validator
        , lighthouse: demand.lighthouse
        , lighthouseFee: 1
        , deadline: await web3.eth.getBlockNumber() + 100000
        , nonce: BigNumber(await factory.methods.nonceOf(robot_account).call()).toNumber()
        , sender: robot_account
    };

    const hash = web3.utils.soliditySha3(
        { t: 'bytes', v: offer.model },
        { t: 'bytes', v: offer.objective },
        { t: 'address', v: offer.token },
        { t: 'uint256', v: offer.cost },
        { t: 'address', v: offer.validator },
        { t: 'address', v: offer.lighthouse },
        { t: 'uint256', v: offer.lighthouseFee },
        { t: 'uint256', v: offer.deadline },
        { t: 'uint256', v: offer.nonce },
        { t: 'address', v: offer.sender }
    );
    offer.signature = await web3.eth.accounts.sign(hash, robot_account_pk);

    return offer;
}

function encodeDemand(demand) {
    return web3.eth.abi.encodeParameters(
        ['bytes'
            , 'bytes'
            , 'address'
            , 'uint256'
            , 'address'
            , 'address'
            , 'uint256'
            , 'uint256'
            , 'address'
            , 'bytes'
        ],
        [demand.model
            , demand.objective
            , demand.token
            , demand.cost
            , demand.lighthouse
            , demand.validator
            , demand.validatorFee
            , demand.deadline
            , demand.sender
            , demand.signature.signature
        ]
    );
}

function encodeOffer(offer) {
    return web3.eth.abi.encodeParameters(
        ['bytes'
            , 'bytes'
            , 'address'
            , 'uint256'
            , 'address'
            , 'address'
            , 'uint256'
            , 'uint256'
            , 'address'
            , 'bytes'
        ],
        [offer.model
            , offer.objective
            , offer.token
            , offer.cost
            , offer.validator
            , offer.lighthouse
            , offer.lighthouseFee
            , offer.deadline
            , offer.sender
            , offer.signature.signature
        ]
    );
}

async function liabilityCreation(web3) {
    let abi = readFileSync(`${args[0]}/liability/abi/Lighthouse.json`)
    let json_abi = JSON.parse(abi)

    let lighthouse = await new web3.eth.Contract(json_abi, lighthouse_address)

    abi = readFileSync(`${args[0]}/liability/abi/Factory.json`)
    json_abi = JSON.parse(abi)

    let factory = await new web3.eth.Contract(json_abi, factory_address)

    let demand = await randomDemand(web3, factory)
    let offer = await pairOffer(demand, web3, factory)

    let d_encoded = encodeDemand(demand)
    let o_encoded = encodeOffer(offer)

    let tx = await lighthouse.methods.createLiability(d_encoded, o_encoded).send({ from: main_account, gas: 1000000000 })
    return tx
}

await web3.eth.accounts.wallet.add(main_account_pk)
await web3.eth.accounts.wallet.add(robot_account_pk)
await web3.eth.accounts.wallet.add(validator_account_pk)
web3.eth.defaultAccount = main_account

const liability_tx = await liabilityCreation(web3)

const liability_receipt = await web3.eth.getTransactionReceipt(liability_tx["transactionHash"])
const liability_address_hex = liability_receipt["logs"][1]["topics"][1]
const liability_address_dec = "0x" + liability_address_hex.slice(26)
const liability_address = web3.utils.toChecksumAddress(liability_address_dec)
console.log(liability_address)